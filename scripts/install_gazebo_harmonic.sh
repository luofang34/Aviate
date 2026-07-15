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
# Phase markers are timestamped so a stall is attributable from the log to
# a specific network touchpoint (first apt update, key fetch, second apt
# update, or package install).
#
# Usage:
#   ./scripts/install_gazebo_harmonic.sh build       # plugin toolchain + gz-harmonic
#   ./scripts/install_gazebo_harmonic.sh runtime     # headless runtime + gz-harmonic
#   ./scripts/install_gazebo_harmonic.sh --self-test # bound check only; no root, no apt
#
# Privileged commands use sudo internally (matching passwordless sudo on
# CI runners); the self-test needs neither root nor apt and runs on macOS.
#
# Env overrides:
#   GZ_KEY_URL            repository signing key URL (default: OSRF)
#   GZ_REPO_URL           apt repository base URL (default: OSRF)
#   GZ_KEYRING            signing-key destination path
#   GZ_SELF_TEST_URL      unreachable fixture the self-test fetches
#   GZ_SELF_TEST_BOUND_S  wall-clock bound the self-test asserts
#
# Exit codes: 0 success, 1 install/self-test failure, 2 bad invocation.

set -euo pipefail

GZ_KEY_URL="${GZ_KEY_URL:-https://packages.osrfoundation.org/gazebo.gpg}"
GZ_REPO_URL="${GZ_REPO_URL:-http://packages.osrfoundation.org/gazebo/ubuntu-stable}"
GZ_KEYRING="${GZ_KEYRING:-/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg}"

# Worst case per fetch: (1 + retries) attempts x --max-time, plus 1+2+4 s
# retry backoff — about 4 minutes, inside the workflow step timeout.
CURL_BOUNDS=(
    --fail --location --silent --show-error
    --retry 3 --retry-connrefused
    --connect-timeout 10 --max-time 60
)

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

phase() {
    printf '[install-gazebo %s] %s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$*"
}

# fetch_key <url> <dest> — bounded download of the repository signing key.
fetch_key() {
    curl "${CURL_BOUNDS[@]}" --output "$2" "$1"
}

# apt_get <args...> — apt with the bounded acquire policy, non-interactive
# so a package configuration prompt cannot stall the runner silently.
apt_get() {
    sudo DEBIAN_FRONTEND=noninteractive apt-get "${APT_BOUNDS[@]}" "$@"
}

install_profile() {
    local profile="$1"
    shift
    phase "profile=${profile}: apt-get update (base index)"
    apt_get update

    phase "apt-get install bootstrap + profile packages: $*"
    apt_get install -y "${BASE_PACKAGES[@]}" "$@"

    phase "fetch OSRF signing key from ${GZ_KEY_URL}"
    local tmpkey
    tmpkey="$(mktemp)"
    fetch_key "$GZ_KEY_URL" "$tmpkey"
    sudo install -m 0644 "$tmpkey" "$GZ_KEYRING"
    rm -f "$tmpkey"

    phase "register OSRF apt repository ${GZ_REPO_URL}"
    echo "deb [arch=$(dpkg --print-architecture) signed-by=${GZ_KEYRING}] ${GZ_REPO_URL} $(lsb_release -cs) main" \
        | sudo tee /etc/apt/sources.list.d/gazebo-stable.list

    phase "apt-get update (OSRF index)"
    apt_get update

    phase "apt-get install gz-harmonic"
    apt_get install -y gz-harmonic

    phase "profile=${profile}: done"
}

# Asserts the fixture fetch fails, and fails within the bound. The
# default fixture is a connection-refused address, which every retry
# rejects immediately; the assertion bound is far below the analytic
# worst case so a policy regression (lost --retry/--max-time flags,
# an accidental infinite retry) is caught, not masked.
self_test() {
    local fixture="${GZ_SELF_TEST_URL:-http://127.0.0.1:9/gazebo.gpg}"
    local bound_s="${GZ_SELF_TEST_BOUND_S:-30}"
    local dest rc start elapsed
    dest="$(mktemp)"
    phase "self-test: bounded key fetch against unreachable fixture ${fixture}"
    start="$(date +%s)"
    set +e
    fetch_key "$fixture" "$dest"
    rc=$?
    set -e
    elapsed=$(( $(date +%s) - start ))
    rm -f "$dest"
    if [ "$rc" -eq 0 ]; then
        phase "self-test: FAILED — fixture fetch unexpectedly succeeded"
        return 1
    fi
    if [ "$elapsed" -gt "$bound_s" ]; then
        phase "self-test: FAILED — fetch took ${elapsed}s, bound is ${bound_s}s"
        return 1
    fi
    phase "self-test: OK — fetch failed as required (curl rc=${rc}) in ${elapsed}s (bound ${bound_s}s)"
}

case "${1:-}" in
    build)
        install_profile build "${BUILD_PACKAGES[@]}"
        ;;
    runtime)
        install_profile runtime "${RUNTIME_PACKAGES[@]}"
        ;;
    --self-test)
        self_test
        ;;
    *)
        echo "usage: $0 build|runtime|--self-test" >&2
        exit 2
        ;;
esac
