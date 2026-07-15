#!/usr/bin/env bash
#
# Bounded Gazebo Harmonic install (gz-sim8 / gz-transport13 / gz-msgs10 /
# gz-plugin2 from the OSRF apt repository) for SITL CI.
#
# Single entry point for every CI job that needs Gazebo, so the repository
# bootstrap, the retry/timeout policy, and the phase markers cannot drift
# apart between jobs. Every network operation carries a finite retry and
# timeout budget: a stalled mirror or key server must fail the step within
# its declared bound instead of holding a required gate until the
# workflow-level timeout.
#
# apt operations consult an explicit source allowlist (the runner's Ubuntu
# archive sources plus the OSRF repository) via a temporary sources
# directory, so an unrelated third-party source preinstalled on the runner
# can neither stall nor fail the install. The runner's real apt
# configuration is never mutated.
#
# Phase markers are timestamped so a stall is attributable from the log to
# a specific network touchpoint (first apt update, key fetch, second apt
# update, or package install), and the allowlisted source set is printed
# so the log proves which sources were consulted.
#
# Usage:
#   ./scripts/install_gazebo_harmonic.sh build       # plugin toolchain + gz-harmonic
#   ./scripts/install_gazebo_harmonic.sh runtime     # headless runtime + gz-harmonic
#   ./scripts/install_gazebo_harmonic.sh --self-test # bound checks only; no root, no apt
#
# The self-test proves both failure modes of the key fetch terminate
# within their bounds: a refused connection (retry/backoff path) and a
# connection that is accepted but never answered (--max-time stall path).
# It needs neither root nor apt and runs on macOS.
#
# Env overrides:
#   GZ_KEY_URL                  repository signing key URL (default: OSRF)
#   GZ_REPO_URL                 apt repository base URL (default: OSRF)
#   GZ_KEYRING                  signing-key destination (default: inside
#                               the temporary allowlist dir)
#   GZ_CURL_RETRIES             curl retry count
#   GZ_CURL_CONNECT_TIMEOUT_S   curl per-attempt connect timeout
#   GZ_CURL_MAX_TIME_S          curl per-attempt total-time cap
#   GZ_SELF_TEST_URL            unreachable fixture, refused subtest
#   GZ_SELF_TEST_BOUND_S        wall-clock bound, refused subtest
#   GZ_SELF_TEST_STALL_MAX_S    scaled-down max-time, stall subtest
#   GZ_SELF_TEST_STALL_BOUND_S  wall-clock bound, stall subtest
#
# Exit codes: 0 success, 1 install/self-test failure, 2 bad invocation.

set -euo pipefail

GZ_KEY_URL="${GZ_KEY_URL:-https://packages.osrfoundation.org/gazebo.gpg}"
GZ_REPO_URL="${GZ_REPO_URL:-http://packages.osrfoundation.org/gazebo/ubuntu-stable}"
GZ_KEYRING="${GZ_KEYRING:-}"

# Bounded-fetch policy for the signing key. Worst case per fetch:
# (1 + retries) attempts x max-time, plus 1+2+4 s retry backoff — about
# 4 minutes at the defaults, inside the workflow step timeout. The values
# are overridable so the self-test can prove stall termination through
# the same fetch path without waiting out the production budget.
GZ_CURL_RETRIES="${GZ_CURL_RETRIES:-3}"
GZ_CURL_CONNECT_TIMEOUT_S="${GZ_CURL_CONNECT_TIMEOUT_S:-10}"
GZ_CURL_MAX_TIME_S="${GZ_CURL_MAX_TIME_S:-60}"

# Acquire::Retries re-tries transient index/package fetch failures;
# the per-protocol timeouts cap how long any single connection may sit
# without progress.
APT_BOUNDS=(
    -o Acquire::Retries=3
    -o Acquire::http::Timeout=30
    -o Acquire::https::Timeout=30
)

# The repository bootstrap itself needs curl / lsb-release / gnupg.
BASE_PACKAGES=(curl lsb-release gnupg)

# Toolchain for the C++ Gazebo plugin and the serialport-backed gcs-test:
# the plugin links protobuf via gz-msgs10; libudev is a transitive
# serialport build dependency.
BUILD_PACKAGES=(
    cmake build-essential
    libudev-dev libprotobuf-dev protobuf-compiler libabsl-dev
)

# Headless mission runtime: xvfb plus a software-GL stack.
RUNTIME_PACKAGES=(xvfb libgl1-mesa-dri libegl1 mesa-utils)

APT_ALLOWLIST_DIR=""

phase() {
    printf '[install-gazebo %s] %s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$*"
}

# fetch_key <url> <dest> — bounded download of the repository signing key:
# --max-time terminates a connection that goes silent mid-transfer;
# --retry / --retry-connrefused bound transient failures.
fetch_key() {
    curl --fail --location --silent --show-error \
        --retry "${GZ_CURL_RETRIES}" --retry-connrefused \
        --connect-timeout "${GZ_CURL_CONNECT_TIMEOUT_S}" \
        --max-time "${GZ_CURL_MAX_TIME_S}" \
        --output "$2" "$1"
}

# apt against the explicit source allowlist only; non-interactive so a
# package configuration prompt cannot stall the runner silently. Default
# trusted-keyring directories stay in effect, so Ubuntu archive signature
# verification is unchanged.
apt_get() {
    sudo DEBIAN_FRONTEND=noninteractive apt-get \
        "${APT_BOUNDS[@]}" \
        -o Dir::Etc::sourcelist=/dev/null \
        -o Dir::Etc::sourceparts="${APT_ALLOWLIST_DIR}" \
        "$@"
}

# Copy only the Ubuntu archive sources into a temporary sources dir.
# Ubuntu archive sources are deb822 (ubuntu.sources) on noble-era runner
# images and a classic sources.list on older ones; take whichever exists,
# and nothing else. Fails closed if neither is found.
setup_apt_allowlist() {
    APT_ALLOWLIST_DIR="$(mktemp -d)"
    chmod 755 "${APT_ALLOWLIST_DIR}"
    local found=0
    if [ -f /etc/apt/sources.list.d/ubuntu.sources ]; then
        cp /etc/apt/sources.list.d/ubuntu.sources "${APT_ALLOWLIST_DIR}/ubuntu.sources"
        found=1
    fi
    if [ -f /etc/apt/sources.list ] && grep -q '^deb ' /etc/apt/sources.list; then
        cp /etc/apt/sources.list "${APT_ALLOWLIST_DIR}/ubuntu-archive.list"
        found=1
    fi
    if [ "${found}" -eq 0 ]; then
        phase "FAILED — no Ubuntu archive source found to allowlist"
        return 1
    fi
    chmod 644 "${APT_ALLOWLIST_DIR}"/*
}

# The log must prove which sources apt consulted.
emit_apt_allowlist() {
    phase "apt source allowlist (apt consults only these):"
    local f
    for f in "${APT_ALLOWLIST_DIR}"/*; do
        printf '  %s\n' "$(basename "$f")"
        grep -E '^(deb |URIs:|Suites:)' "$f" | sed 's/^/    /' || true
    done
}

install_profile() {
    local profile="$1"
    shift
    setup_apt_allowlist
    local keyring="${GZ_KEYRING:-${APT_ALLOWLIST_DIR}/pkgs-osrf-archive-keyring.gpg}"
    emit_apt_allowlist

    phase "profile=${profile}: apt-get update (Ubuntu archive index)"
    apt_get update

    phase "apt-get install bootstrap + profile packages: $*"
    apt_get install -y "${BASE_PACKAGES[@]}" "$@"

    phase "fetch OSRF signing key from ${GZ_KEY_URL}"
    local tmpkey
    tmpkey="$(mktemp)"
    fetch_key "${GZ_KEY_URL}" "${tmpkey}"
    install -m 0644 "${tmpkey}" "${keyring}"
    rm -f "${tmpkey}"

    phase "register OSRF apt repository ${GZ_REPO_URL} (allowlist only; runner apt config untouched)"
    echo "deb [arch=$(dpkg --print-architecture) signed-by=${keyring}] ${GZ_REPO_URL} $(lsb_release -cs) main" \
        > "${APT_ALLOWLIST_DIR}/gazebo-stable.list"
    chmod 644 "${APT_ALLOWLIST_DIR}/gazebo-stable.list"
    emit_apt_allowlist

    phase "apt-get update (Ubuntu + OSRF indexes)"
    apt_get update

    phase "apt-get install gz-harmonic"
    apt_get install -y gz-harmonic

    phase "profile=${profile}: done"
}

# Asserts a refused connection fails, and fails within the bound. Every
# retry is rejected immediately, so the assertion bound sits far below
# the analytic worst case: a policy regression (lost --retry flags, an
# accidental infinite retry) is caught, not masked.
self_test_refused() {
    local fixture="${GZ_SELF_TEST_URL:-http://127.0.0.1:9/gazebo.gpg}"
    local bound_s="${GZ_SELF_TEST_BOUND_S:-30}"
    local dest rc start elapsed
    dest="$(mktemp)"
    phase "self-test(refused): bounded key fetch against unreachable fixture ${fixture}"
    start="$(date +%s)"
    set +e
    fetch_key "${fixture}" "${dest}"
    rc=$?
    set -e
    elapsed=$(( $(date +%s) - start ))
    rm -f "${dest}"
    if [ "${rc}" -eq 0 ]; then
        phase "self-test(refused): FAILED — fixture fetch unexpectedly succeeded"
        return 1
    fi
    if [ "${elapsed}" -gt "${bound_s}" ]; then
        phase "self-test(refused): FAILED — fetch took ${elapsed}s, bound is ${bound_s}s"
        return 1
    fi
    phase "self-test(refused): OK — fetch failed as required (curl rc=${rc}) in ${elapsed}s (bound ${bound_s}s)"
}

# Asserts --max-time terminates a connection that is accepted and then
# never answered — retry/backoff alone cannot detect this mode. The
# fixture scales max-time down through the same fetch path, so the test
# proves termination of a genuine stall without waiting out the
# production budget; the production values are the defaults of the same
# policy. The elapsed floor proves the transfer really stalled instead
# of failing fast.
self_test_stalled() {
    if ! command -v python3 >/dev/null 2>&1; then
        phase "self-test(stall): FAILED — python3 unavailable for the stall fixture"
        return 1
    fi
    local max_s="${GZ_SELF_TEST_STALL_MAX_S:-5}"
    local bound_s="${GZ_SELF_TEST_STALL_BOUND_S:-25}"
    local port_file dest server_pid port rc start elapsed
    port_file="$(mktemp)"
    dest="$(mktemp)"
    # Accept every connection (each curl retry opens a new one), keep
    # the sockets open, never send a byte.
    python3 - >"${port_file}" <<'PYEOF' &
import socket, time
srv = socket.socket()
srv.bind(("127.0.0.1", 0))
srv.listen(4)
print(srv.getsockname()[1], flush=True)
deadline = time.time() + 90
conns = []
srv.settimeout(1.0)
while time.time() < deadline:
    try:
        conns.append(srv.accept()[0])
    except OSError:
        pass
PYEOF
    server_pid=$!
    for _ in $(seq 1 50); do
        [ -s "${port_file}" ] && break
        sleep 0.1
    done
    port="$(head -n1 "${port_file}")"
    if [ -z "${port}" ]; then
        phase "self-test(stall): FAILED — fixture server did not report a port"
        kill "${server_pid}" 2>/dev/null || true
        rm -f "${port_file}" "${dest}"
        return 1
    fi
    phase "self-test(stall): key fetch from accept-then-stall fixture 127.0.0.1:${port} (max-time ${max_s}s, 1 retry)"
    start="$(date +%s)"
    set +e
    (
        GZ_CURL_MAX_TIME_S="${max_s}"
        GZ_CURL_RETRIES=1
        fetch_key "http://127.0.0.1:${port}/gazebo.gpg" "${dest}"
    )
    rc=$?
    set -e
    elapsed=$(( $(date +%s) - start ))
    kill "${server_pid}" 2>/dev/null || true
    wait "${server_pid}" 2>/dev/null || true
    rm -f "${port_file}" "${dest}"
    if [ "${rc}" -eq 0 ]; then
        phase "self-test(stall): FAILED — stalled fetch unexpectedly succeeded"
        return 1
    fi
    if [ "${elapsed}" -lt "${max_s}" ]; then
        phase "self-test(stall): FAILED — fetch returned in ${elapsed}s, before the stall engaged"
        return 1
    fi
    if [ "${elapsed}" -gt "${bound_s}" ]; then
        phase "self-test(stall): FAILED — stalled fetch took ${elapsed}s, bound is ${bound_s}s"
        return 1
    fi
    phase "self-test(stall): OK — stalled fetch terminated (curl rc=${rc}) in ${elapsed}s (bound ${bound_s}s)"
}

case "${1:-}" in
    build)
        install_profile build "${BUILD_PACKAGES[@]}"
        ;;
    runtime)
        install_profile runtime "${RUNTIME_PACKAGES[@]}"
        ;;
    --self-test)
        self_test_refused
        self_test_stalled
        ;;
    *)
        echo "usage: $0 build|runtime|--self-test" >&2
        exit 2
        ;;
esac
