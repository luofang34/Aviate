#!/usr/bin/env bash
#
# Identity adjudication for production algorithm implementations.
#
# Algorithm identity is the cross-channel lockstep witness: two
# channels compare `KernelPipeline::algorithm_identity_hash`, an FNV
# fold of the four substitutable impls' `ALGORITHM_ID` constants, to
# decide whether they run behaviorally compatible code. This gate
# exists so that no behavior-relevant change to a production
# estimator / controller / mixer / sanitizer implementation can land
# without either rotating the identity of THAT implementation in
# cert/algorithm_id_registry.toml, or a human stating — per commit,
# as an exact git trailer — why behavior cannot have changed:
#
#   Algorithm-Identity-Unchanged: <why this cannot change observable behavior>
#
# The trailer must be parsed by `git interpret-trailers` from the
# final trailer block (embedded look-alikes in the body do not
# count), appear exactly once, and carry a non-empty rationale.
#
# Beyond the range adjudication the script proves, at the head
# revision, that the identity ledger is internally coherent:
#
#   * every active ID is globally unique, including [testing] IDs;
#   * no active ID reuses a retired ID (retired IDs are the hex
#     literals quoted in registry comments — never quote an active
#     ID in a comment);
#   * every production implementation's compiled `ALGORITHM_ID`
#     constant equals its registry entry (IMPL_MAP parity);
#   * every `ALGORITHM_ID` literal anywhere in the repo is
#     registered: production files must use production-section IDs,
#     test files must use [testing] IDs;
#   * the production aggregate identity hashes (generic quad-X and
#     X500 bundles) equal the pinned values, which are the same
#     values TST-PIPE-104 pins on the Rust side;
#   * across a range, every ID active at base is still active or
#     explicitly retired at head (no silent ledger drops).
#
# Usage:
#   scripts/check_algorithm_identity.sh <base-rev> <head-rev>
#   scripts/check_algorithm_identity.sh --structural <rev>
#   scripts/check_algorithm_identity.sh --self-test
#
# Exit codes: 0 adjudicated / coherent, 1 adjudication or coherence
# failure, 2 bad invocation.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# ALGID_REPO_ROOT points the checks at another checkout (fixture
# debugging); CI never sets it, so the gate always reads its own
# checkout there.
REPO_ROOT="${ALGID_REPO_ROOT:-$(cd "$SCRIPT_DIR/.." && pwd)}"

REGISTRY='cert/algorithm_id_registry.toml'
TRAILER_KEY='Algorithm-Identity-Unchanged'
PROD_SECTIONS='estimator controller mixer sanitizer'

# Machine-readable mapping: every production implementation, its
# active registry entry, and the file whose `impl <Trait> for <Type>`
# block declares the compiled `ALGORITHM_ID` constant.
# Fields: section|registry-key|rust-type|const-file
IMPL_MAP=(
    'estimator|ekf.basic-15state.v3|Ekf|aviate-core/src/ekf/scalar.rs'
    'controller|controller.multirotor.v2|MultirotorController|aviate-core/src/control/multirotor.rs'
    'controller|controller.fixed_wing.v1|FixedWingController|aviate-core/src/control/fixed_wing.rs'
    'controller|controller.vtol.v1|VtolController|aviate-core/src/control/vtol.rs'
    'mixer|mixer.quad_x.v2|QuadXMixer|aviate-core/src/mixer.rs'
    'mixer|mixer.quad_x_x500.v2|QuadXMixerX500|aviate-core/src/mixer.rs'
    'sanitizer|sanitizer.group_aware.v1|Sanitizer|aviate-core/src/mixer/sanitizer_impl.rs'
)

# Ownership: which registry entries adjudicate a change to a given
# production file. First match wins; a trailing '/' is a tree prefix.
# A file owned by several entries (shared code) is adjudicated by a
# rotation of ANY of its owners — an entry that does not own the file
# never satisfies the gate for it. The sanitizer tree is explicit so
# a sanitizer change is adjudicated against the sanitizer identity,
# not waved through as generic mixer-tree churn.
# Fields: pattern|comma-separated owners (section/key)
OWNERSHIP=(
    'aviate-core/src/ekf.rs|estimator/ekf.basic-15state.v3'
    'aviate-core/src/ekf/|estimator/ekf.basic-15state.v3'
    'aviate-core/src/control/multirotor.rs|controller/controller.multirotor.v2'
    'aviate-core/src/control/fixed_wing.rs|controller/controller.fixed_wing.v1'
    'aviate-core/src/control/vtol.rs|controller/controller.vtol.v1'
    'aviate-core/src/control.rs|controller/controller.multirotor.v2,controller/controller.fixed_wing.v1,controller/controller.vtol.v1'
    'aviate-core/src/control/|controller/controller.multirotor.v2,controller/controller.fixed_wing.v1,controller/controller.vtol.v1'
    'aviate-core/src/mixer/sanitizer_impl.rs|sanitizer/sanitizer.group_aware.v1'
    'aviate-core/src/mixer/desaturate.rs|mixer/mixer.quad_x.v2,mixer/mixer.quad_x_x500.v2'
    'aviate-core/src/mixer.rs|mixer/mixer.quad_x.v2,mixer/mixer.quad_x_x500.v2,sanitizer/sanitizer.group_aware.v1'
    'aviate-core/src/mixer/|mixer/mixer.quad_x.v2,mixer/mixer.quad_x_x500.v2,sanitizer/sanitizer.group_aware.v1'
)

# Pinned production aggregate identity hashes. These are the same
# FNV-folded values TST-PIPE-104 pins in
# aviate-core/src/kernel/pipeline.rs, computed here from the registry
# instead of from compiled constants, so registry and source cannot
# drift apart without both pins moving in the same commit.
# Fold order matches KernelPipeline::algorithm_identity_hash:
# estimator, controller, mixer, sanitizer.
GENERIC_QUAD_BUNDLE='ekf.basic-15state.v3 controller.multirotor.v2 mixer.quad_x.v2 sanitizer.group_aware.v1'
GENERIC_QUAD_AGGREGATE='646b55c0745dab84'
X500_BUNDLE='ekf.basic-15state.v3 controller.multirotor.v2 mixer.quad_x_x500.v2 sanitizer.group_aware.v1'
X500_AGGREGATE='20ce8c48728724d5'

# ---------------------------------------------------------------- utils

# Normalize a 0x… Rust/TOML hex literal to 16 lowercase digits.
norm_hex() {
    local h="$1"
    h="${h#0x}"
    h="${h//_/}"
    h="$(printf '%s' "$h" | tr 'A-F' 'a-f')"
    printf '%016s' "$h" | tr ' ' '0'
}

# FNV-1a-style fold over u64 IDs, 8 little-endian bytes each —
# byte-for-byte the fold in KernelPipeline::algorithm_identity_hash.
# Bash arithmetic is 64-bit two's complement, so the multiply wraps
# exactly like `wrapping_mul` does.
fnv_fold() {
    local hash=$(( 0xcbf29ce484222325 ))
    local prime=$(( 0x100000001b3 ))
    local id_hex id byte i
    for id_hex in "$@"; do
        id=$(( 16#$id_hex ))
        for i in 0 1 2 3 4 5 6 7; do
            byte=$(( (id >> (8*i)) & 0xff ))
            hash=$(( hash ^ byte ))
            hash=$(( hash * prime ))
        done
    done
    printf '%016x' "$hash"
}

# Is this path test-only code (cannot change flight behavior)?
is_test_path() {
    case "$1" in
        *tests.rs | */tests/* ) return 0 ;;
        * ) return 1 ;;
    esac
}

# Print the comma-separated owners of a production file, or nothing
# if the file is not production algorithm code.
owners_for() {
    local file="$1" rule pattern owners
    for rule in "${OWNERSHIP[@]}"; do
        pattern="${rule%%|*}"
        owners="${rule#*|}"
        case "$pattern" in
            */ ) [[ "$file" == "$pattern"* ]] && { printf '%s' "$owners"; return 0; } ;;
            *  ) [[ "$file" == "$pattern"  ]] && { printf '%s' "$owners"; return 0; } ;;
        esac
    done
    return 0
}

# ------------------------------------------------------- registry parse

# Emit `A <section> <key> <hex16>` for active entries and `R <hex16>`
# for every hex literal quoted in a comment (the retired/reserved
# set), reading the registry at a specific revision.
registry_parse() {
    local rev="$1"
    git -C "$REPO_ROOT" show "$rev:$REGISTRY" | awk '
        /^\[/ {
            section = $0
            gsub(/[][]/, "", section)
            next
        }
        /^"/ {
            key = $0
            sub(/^"/, "", key); sub(/".*/, "", key)
            if (match($0, /0x[0-9A-Fa-f_]+/)) {
                print "A " section " " key " " substr($0, RSTART, RLENGTH)
            }
            next
        }
        /^[ \t]*#/ {
            line = $0
            while (match(line, /0x[0-9A-Fa-f_]+/)) {
                print "R " substr(line, RSTART, RLENGTH)
                line = substr(line, RSTART + RLENGTH)
            }
        }
    ' | while IFS=' ' read -r kind f1 f2 f3; do
        case "$kind" in
            A) printf 'A %s %s %s\n' "$f1" "$f2" "$(norm_hex "$f3")" ;;
            R) printf 'R %s\n' "$(norm_hex "$f1")" ;;
        esac
    done
}

# Look up an active entry's value in a registry_parse dump.
# registry_value <dump> <section> <key> → hex16 (empty if absent)
registry_value() {
    local dump="$1" section="$2" key="$3"
    awk -v s="$section" -v k="$key" '$1 == "A" && $2 == s && $3 == k { print $4; exit }' <<< "$dump"
}

# Extract the compiled ALGORITHM_ID declared inside `impl … for
# <Type>` in a file at a revision. Relies on rustfmt closing
# top-level impl blocks at column 0.
compiled_id() {
    local rev="$1" file="$2" type="$3" raw
    raw="$(git -C "$REPO_ROOT" show "$rev:$file" 2>/dev/null | awk -v type="$type" '
        $0 ~ ("^impl[^{]* for (.*::)?" type "([^A-Za-z0-9_]|$)") { in_impl = 1 }
        in_impl && /^\}/ { in_impl = 0 }
        in_impl && match($0, /const[ \t]+ALGORITHM_ID[ \t]*:[ \t]*u64[ \t]*=[ \t]*0x[0-9A-Fa-f_]+/) {
            s = substr($0, RSTART, RLENGTH)
            sub(/.*0x/, "0x", s)
            print s
            exit
        }
    ')"
    [[ -n "$raw" ]] && norm_hex "$raw"
    return 0
}

# ---------------------------------------------------- structural checks

# Prove the ledger, the compiled constants, and the pinned aggregates
# agree at one revision. Prints failures; returns 1 on any.
structural_check() {
    local rev="$1" bad=0 dump
    if ! dump="$(registry_parse "$rev")" || [[ -z "$dump" ]]; then
        echo "FAIL: cannot read $REGISTRY at $rev" >&2
        return 1
    fi

    # Active IDs are globally unique (production and testing alike).
    local dups
    dups="$(awk '$1 == "A" { print $4 }' <<< "$dump" | sort | uniq -d)"
    if [[ -n "$dups" ]]; then
        echo "FAIL: duplicate active ALGORITHM_IDs in $REGISTRY:" >&2
        local d
        while IFS= read -r d; do
            awk -v v="$d" '$1 == "A" && $4 == v { printf "  0x%s = [%s] %s\n", v, $2, $3 }' <<< "$dump" >&2
        done <<< "$dups"
        bad=1
    fi

    # No active ID reuses a retired one. Retired IDs must stay quoted
    # in registry comments forever; an active ID equal to any quoted
    # hex is a reuse (or a comment quoting a live ID — also banned,
    # so the reserved set stays unambiguous).
    local reused
    reused="$(comm -12 \
        <(awk '$1 == "A" { print $4 }' <<< "$dump" | sort -u) \
        <(awk '$1 == "R" { print $2 }' <<< "$dump" | sort -u))"
    if [[ -n "$reused" ]]; then
        echo "FAIL: active ALGORITHM_ID reuses a retired/quoted ID:" >&2
        sed 's/^/  0x/' <<< "$reused" >&2
        bad=1
    fi

    # Every production-section entry has an IMPL_MAP owner: an entry
    # nobody implements is either dead or a mapping gap, and both
    # break the registry-to-source parity story.
    local section key
    while IFS=' ' read -r _ section key _; do
        case " $PROD_SECTIONS " in *" $section "*) ;; *) continue ;; esac
        local found=0 row
        for row in "${IMPL_MAP[@]}"; do
            IFS='|' read -r msec mkey _ _ <<< "$row"
            if [[ "$msec" == "$section" && "$mkey" == "$key" ]]; then
                found=1
                break
            fi
        done
        if [[ $found -eq 0 ]]; then
            echo "FAIL: registry entry [$section] \"$key\" has no IMPL_MAP row (unmapped production identity)" >&2
            bad=1
        fi
    done < <(awk '$1 == "A"' <<< "$dump")

    # Registry ↔ compiled-constant parity, and table self-consistency:
    # the const file must be owned by the entry it implements.
    local row msec mkey mtype mfile reg_id src_id
    for row in "${IMPL_MAP[@]}"; do
        IFS='|' read -r msec mkey mtype mfile <<< "$row"
        reg_id="$(registry_value "$dump" "$msec" "$mkey")"
        if [[ -z "$reg_id" ]]; then
            echo "FAIL: IMPL_MAP names [$msec] \"$mkey\" but the registry at $rev has no such active entry" >&2
            bad=1
            continue
        fi
        src_id="$(compiled_id "$rev" "$mfile" "$mtype")"
        if [[ -z "$src_id" ]]; then
            echo "FAIL: no ALGORITHM_ID constant found for $mtype in $mfile at $rev" >&2
            bad=1
        elif [[ "$src_id" != "$reg_id" ]]; then
            echo "FAIL: identity mismatch for [$msec] \"$mkey\": registry 0x$reg_id vs compiled 0x$src_id ($mtype in $mfile)" >&2
            bad=1
        fi
        case ",$(owners_for "$mfile")," in
            *",$msec/$mkey,"*) ;;
            *)
                echo "FAIL: OWNERSHIP does not assign $mfile to $msec/$mkey (mapping tables disagree)" >&2
                bad=1
                ;;
        esac
    done

    # Every ALGORITHM_ID literal in the repo is registered: production
    # files must carry production-section IDs, test files must carry
    # [testing] IDs (test-only identities stay outside the production
    # allocation), and anything else must at least be registered so it
    # is collision-checked.
    local occ file lit lit_norm
    while IFS= read -r occ; do
        [[ -z "$occ" ]] && continue
        occ="${occ#"$rev":}"
        file="${occ%%:*}"
        lit="$(grep -oE '0x[0-9A-Fa-f_]+' <<< "$occ" | head -1)"
        [[ -z "$lit" ]] && continue
        lit_norm="$(norm_hex "$lit")"
        if is_test_path "$file"; then
            if ! awk -v v="$lit_norm" '$1 == "A" && $2 == "testing" && $4 == v { found = 1 } END { exit !found }' <<< "$dump"; then
                echo "FAIL: test-only ALGORITHM_ID 0x$lit_norm in $file is not registered under [testing]" >&2
                bad=1
            fi
        elif [[ -n "$(owners_for "$file")" ]]; then
            if ! awk -v v="$lit_norm" -v secs=" $PROD_SECTIONS " '$1 == "A" && index(secs, " " $2 " ") && $4 == v { found = 1 } END { exit !found }' <<< "$dump"; then
                echo "FAIL: production ALGORITHM_ID 0x$lit_norm in $file has no production-section registry entry" >&2
                bad=1
            fi
        else
            if ! awk -v v="$lit_norm" '$1 == "A" && $4 == v { found = 1 } END { exit !found }' <<< "$dump"; then
                echo "FAIL: ALGORITHM_ID 0x$lit_norm in $file is not registered at all" >&2
                bad=1
            fi
        fi
    done < <(git -C "$REPO_ROOT" grep -I -E 'const[ \t]+ALGORITHM_ID[ \t]*:[ \t]*u64[ \t]*=[ \t]*0x' "$rev" -- '*.rs' 2>/dev/null || true)

    # Pinned production aggregates. The generic pin doubles as a
    # cross-check that this script's fold matches the Rust fold, via
    # the shared TST-PIPE-104 constant.
    if ! check_aggregate "$dump" 'generic quad-X' "$GENERIC_QUAD_BUNDLE" "$GENERIC_QUAD_AGGREGATE"; then
        bad=1
    fi
    if ! check_aggregate "$dump" 'X500' "$X500_BUNDLE" "$X500_AGGREGATE"; then
        bad=1
    fi

    return "$bad"
}

# check_aggregate <dump> <name> <bundle keys> <pinned hex16>
check_aggregate() {
    local dump="$1" name="$2" bundle="$3" pinned="$4"
    local ids=() key val
    for key in $bundle; do
        val="$(awk -v k="$key" '$1 == "A" && $3 == k { print $4; exit }' <<< "$dump")"
        if [[ -z "$val" ]]; then
            echo "FAIL: $name bundle member \"$key\" missing from registry" >&2
            return 1
        fi
        ids+=("$val")
    done
    local actual
    actual="$(fnv_fold "${ids[@]}")"
    if [[ "$actual" != "$pinned" ]]; then
        echo "FAIL: $name aggregate identity drifted: pinned 0x$pinned, registry folds to 0x$actual." >&2
        echo "  A bundle member rotated: move this pin and the TST-PIPE-104 pin in the same commit." >&2
        return 1
    fi
    return 0
}

# Every ID active at base must still be active (any key) or quoted as
# retired at head — identities never silently vanish from the ledger.
continuity_check() {
    local base="$1" head="$2" bad=0
    local base_dump head_dump
    base_dump="$(registry_parse "$base" 2>/dev/null || true)"
    [[ -z "$base_dump" ]] && return 0
    head_dump="$(registry_parse "$head")"
    local dropped
    dropped="$(comm -23 \
        <(awk '$1 == "A" { print $4 }' <<< "$base_dump" | sort -u) \
        <(awk '{ print ($1 == "A") ? $4 : $2 }' <<< "$head_dump" | sort -u))"
    if [[ -n "$dropped" ]]; then
        echo "FAIL: IDs active at $base vanished at $head without a retired-comment record:" >&2
        sed 's/^/  0x/' <<< "$dropped" >&2
        bad=1
    fi
    return "$bad"
}

# ------------------------------------------------------------- trailers

# A commit is trailer-adjudicated iff `git interpret-trailers` parses
# exactly one exact-key trailer from the final trailer block and its
# rationale is non-empty. Substring matches in the body, look-alike
# keys, duplicates, and bare keys all fail.
trailer_ok() {
    local sha="$1" body parsed count value
    body="$(git -C "$REPO_ROOT" log -1 --format=%B "$sha")"
    parsed="$(printf '%s\n' "$body" | git interpret-trailers --parse 2>/dev/null || true)"
    count="$(grep -c "^${TRAILER_KEY}:" <<< "$parsed" || true)"
    if [[ "$count" -ne 1 ]]; then
        return 1
    fi
    value="$(sed -n "s/^${TRAILER_KEY}:[[:space:]]*//p" <<< "$parsed" | tr -d '[:space:]')"
    [[ -n "$value" ]]
}

# --------------------------------------------------------- adjudication

# adjudicate <base> <head>
adjudicate() {
    local base="$1" head="$2" bad=0

    structural_check "$head" || bad=1
    continuity_check "$base" "$head" || bad=1

    local changed
    changed="$(git -C "$REPO_ROOT" diff --name-only "$base" "$head")"

    # owner|file pairs for every changed production file.
    local pairs='' file owners
    while IFS= read -r file; do
        [[ -z "$file" ]] && continue
        is_test_path "$file" && continue
        owners="$(owners_for "$file")"
        [[ -z "$owners" ]] && continue
        local o
        while IFS= read -r o; do
            pairs+="$o|$file"$'\n'
        done < <(tr ',' '\n' <<< "$owners")
    done <<< "$changed"

    if [[ -z "$pairs" ]]; then
        if [[ $bad -eq 0 ]]; then
            echo "No production algorithm path changed; ledger coherent at $head."
            return 0
        fi
        return 1
    fi

    local base_dump head_dump
    base_dump="$(registry_parse "$base" 2>/dev/null || true)"
    head_dump="$(registry_parse "$head")"

    # A changed file is adjudicated when any one of its owners rotated
    # in this range; otherwise every commit touching it must carry a
    # valid trailer.
    local touched_files unadjudicated=''
    touched_files="$(cut -d'|' -f2 <<< "$pairs" | sort -u)"
    while IFS= read -r file; do
        [[ -z "$file" ]] && continue
        local rotated=0 owner section key base_val head_val
        while IFS= read -r owner; do
            [[ -z "$owner" ]] && continue
            section="${owner%%/*}"
            key="${owner#*/}"
            head_val="$(registry_value "$head_dump" "$section" "$key")"
            base_val="$(registry_value "$base_dump" "$section" "$key")"
            # Rotated: the entry this implementation now points at is
            # new in this range, or changed value in place.
            if [[ -n "$head_val" && ( -z "$base_val" || "$base_val" != "$head_val" ) ]]; then
                echo "Identity rotated for $file: [$section] \"$key\" moved in this range."
                rotated=1
                break
            fi
        done < <(awk -F'|' -v f="$file" '$2 == f { print $1 }' <<< "$pairs" | sort -u)
        if [[ $rotated -eq 0 ]]; then
            unadjudicated+="$file"$'\n'
        fi
    done <<< "$touched_files"

    if [[ -n "$unadjudicated" ]]; then
        # Per-commit trailer fallback, exact and fail-closed: every
        # commit in the range that touches a non-rotated production
        # file must carry the trailer.
        local -a paths=()
        while IFS= read -r file; do
            [[ -n "$file" ]] && paths+=("$file")
        done <<< "$unadjudicated"
        local commits sha missing=''
        commits="$(git -C "$REPO_ROOT" log --format=%H "$base..$head" -- "${paths[@]}" 2>/dev/null || true)"
        if [[ -z "$commits" ]]; then
            echo "FAIL: production files changed in $base..$head but no commit claims them (history rewrite?); cannot adjudicate:" >&2
            printf '  %s\n' "${paths[@]}" >&2
            bad=1
        else
            while IFS= read -r sha; do
                [[ -z "$sha" ]] && continue
                if ! trailer_ok "$sha"; then
                    missing+="$sha"$'\n'
                fi
            done <<< "$commits"
            if [[ -n "$missing" ]]; then
                echo "FAIL: production algorithm files changed without identity rotation:" >&2
                printf '  %s\n' "${paths[@]}" >&2
                echo "and these commits touch them without a valid '$TRAILER_KEY' trailer" >&2
                echo "(exact final trailer, exactly once, non-empty rationale):" >&2
                printf '%s' "$missing" | sed 's/^/  /' >&2
                echo "Either rotate the owning identity in $REGISTRY or add, per commit:" >&2
                echo "  $TRAILER_KEY: <why this cannot change observable behavior>" >&2
                bad=1
            else
                echo "Identity adjudicated: every commit touching non-rotated production files carries an exact $TRAILER_KEY trailer."
            fi
        fi
    fi

    if [[ $bad -eq 0 ]]; then
        echo "Identity adjudication passed for $base..$head."
    fi
    return "$bad"
}

# ------------------------------------------------------------ self-test

# The adversarial fixture: a throwaway repo seeded with the real
# registry and the real implementation files, mutated per scenario.
# Every scenario exercises the actual git-range entry point.
FIXTURE=''

fixture_init() {
    FIXTURE="$(mktemp -d "${TMPDIR:-/tmp}/algid-selftest.XXXXXX")"
    git -C "$FIXTURE" init -q -b main
    git -C "$FIXTURE" config user.email 'selftest@invalid'
    git -C "$FIXTURE" config user.name 'Identity Self-Test'
    git -C "$FIXTURE" config commit.gpgsign false

    local f
    for f in \
        "$REGISTRY" \
        aviate-core/src/ekf/scalar.rs \
        aviate-core/src/ekf/update.rs \
        aviate-core/src/control/multirotor.rs \
        aviate-core/src/control/fixed_wing.rs \
        aviate-core/src/control/vtol.rs \
        aviate-core/src/mixer.rs \
        aviate-core/src/mixer/sanitizer_impl.rs; do
        mkdir -p "$FIXTURE/$(dirname "$f")"
        cp "$REPO_ROOT/$f" "$FIXTURE/$f"
    done
    mkdir -p "$FIXTURE/aviate-link/src"
    echo '// non-production code' > "$FIXTURE/aviate-link/src/queue.rs"
    git -C "$FIXTURE" add -A
    git -C "$FIXTURE" commit -qm 'fixture base'
}

fixture_cleanup() {
    [[ -n "$FIXTURE" && -d "$FIXTURE" ]] && rm -rf "$FIXTURE"
    return 0
}

fixture_branch() {
    git -C "$FIXTURE" checkout -q main
    git -C "$FIXTURE" checkout -qb "$1"
}

fixture_commit() {
    local msg="$1"
    git -C "$FIXTURE" add -A
    git -C "$FIXTURE" commit -qm "$msg"
}

# sed-in-place without GNU/BSD -i divergence.
replace_in() {
    local f="$FIXTURE/$1" old="$2" new="$3"
    local tmp="$f.tmp"
    sed "s/$old/$new/g" "$f" > "$tmp"
    mv "$tmp" "$f"
}

append_line() {
    printf '%s\n' "$2" >> "$FIXTURE/$1"
}

# expect <pass|fail> <name> — runs adjudicate main..HEAD on the
# fixture and compares the verdict.
SELF_TEST_FAILURES=0
expect() {
    local want="$1" name="$2" got
    if (REPO_ROOT="$FIXTURE"; adjudicate main HEAD) > /dev/null 2>&1; then
        got='pass'
    else
        got='fail'
    fi
    if [[ "$got" != "$want" ]]; then
        echo "SELF-TEST FAIL: $name — expected $want, got $got" >&2
        (REPO_ROOT="$FIXTURE"; adjudicate main HEAD) 2>&1 | sed 's/^/    /' >&2 || true
        SELF_TEST_FAILURES=$(( SELF_TEST_FAILURES + 1 ))
    else
        echo "  ok: $name ($want)"
    fi
}

self_test() {
    # The fold must reproduce the Rust-side TST-PIPE-104 pin before
    # anything else is trusted.
    local folded
    folded="$(fnv_fold 4554494d454b4634 43544c4d55525632 4d49585155414432 53414e4752505631)"
    if [[ "$folded" != "$GENERIC_QUAD_AGGREGATE" ]]; then
        echo "SELF-TEST FAIL: bash FNV fold ($folded) disagrees with the TST-PIPE-104 pin ($GENERIC_QUAD_AGGREGATE)" >&2
        return 1
    fi
    echo "  ok: FNV fold matches the Rust TST-PIPE-104 pin"

    trap fixture_cleanup EXIT
    fixture_init

    # Pristine fixture: real registry + real sources must be coherent.
    fixture_branch s-pristine
    append_line aviate-link/src/queue.rs '// non-production touch'
    fixture_commit 'non-production change'
    expect pass 'non-production change needs no adjudication'

    fixture_branch s-tests-only
    mkdir -p "$FIXTURE/aviate-core/src/ekf"
    echo '// test-module change' > "$FIXTURE/aviate-core/src/ekf/tests.rs"
    fixture_commit 'test module change'
    expect pass 'test-module change under a production tree is exempt'

    fixture_branch s-bare
    append_line aviate-core/src/ekf/update.rs '// probe'
    fixture_commit 'unadjudicated EKF change'
    expect fail 'production change without rotation or trailer'

    fixture_branch s-registry-comment
    append_line aviate-core/src/ekf/update.rs '// probe'
    append_line "$REGISTRY" '# audit note, no identity change'
    fixture_commit 'EKF change plus registry comment edit'
    expect fail 'unrelated comment-only registry touch does not adjudicate'

    fixture_branch s-wrong-entry
    append_line aviate-core/src/ekf/update.rs '// probe'
    replace_in "$REGISTRY" '544F_4C31' '544F_4C32'
    replace_in aviate-core/src/control/vtol.rs '544F_4C31' '544F_4C32'
    append_line "$REGISTRY" '# Retired: "controller.vtol.v1" = 0x4354_4C56_544F_4C31.'
    fixture_commit 'EKF change hidden behind a genuine vtol rotation'
    expect fail 'rotating a non-owning entry does not adjudicate an EKF change'

    fixture_branch s-right-entry
    replace_in "$REGISTRY" '544F_4C31' '544F_4C32'
    replace_in aviate-core/src/control/vtol.rs '544F_4C31' '544F_4C32'
    append_line "$REGISTRY" '# Retired: "controller.vtol.v1" = 0x4354_4C56_544F_4C31.'
    fixture_commit 'vtol behavior change with its own rotation'
    expect pass 'rotating the owning entry adjudicates that implementation'

    fixture_branch s-reuse
    replace_in "$REGISTRY" '0x4354_4C56_544F_4C31' '0x4554_494D_454B_4633'
    replace_in aviate-core/src/control/vtol.rs '0x4354_4C56_544F_4C31' '0x4554_494D_454B_4633'
    append_line "$REGISTRY" '# Retired: "controller.vtol.v1" previously held 0x4354_4C56_544F_4C31.'
    fixture_commit 'vtol rotation onto a retired EKF ID'
    expect fail 'reusing a retired ID is rejected'

    fixture_branch s-duplicate
    replace_in "$REGISTRY" '0x4553_544D_4F43_4B00' '0x5341_4E47_5250_5631'
    fixture_commit 'testing entry duplicated onto the sanitizer ID'
    expect fail 'duplicate active IDs are rejected (testing vs production)'

    fixture_branch s-mismatch
    replace_in aviate-core/src/ekf/scalar.rs '454B_4634' '454B_4635'
    fixture_commit 'EKF constant changed without registry'
    expect fail 'source/registry parity mismatch is rejected'

    fixture_branch s-dropped
    replace_in "$REGISTRY" '^"controller.vtol.v1".*$' ''
    fixture_commit 'vtol entry silently deleted'
    expect fail 'silently dropping an active ID from the ledger is rejected'

    fixture_branch s-x500-drift
    replace_in "$REGISTRY" '5835_5632' '5835_5633'
    replace_in aviate-core/src/mixer.rs '5835_5632' '5835_5633'
    append_line "$REGISTRY" '# Retired: "mixer.quad_x_x500.v2" = 0x4D49_5851_5835_5632.'
    fixture_commit 'X500 mixer rotation without moving the aggregate pin'
    expect fail 'X500 aggregate drift is rejected until the pin moves'

    fixture_branch s-trailer-good
    append_line aviate-core/src/ekf/update.rs '// probe'
    git -C "$FIXTURE" add -A
    git -C "$FIXTURE" commit -qm 'Touch EKF comment' \
        -m 'Algorithm-Identity-Unchanged: comment-only change, no executable difference'
    expect pass 'exact final trailer with rationale adjudicates'

    fixture_branch s-trailer-embedded
    append_line aviate-core/src/ekf/update.rs '// probe'
    git -C "$FIXTURE" add -A
    git -C "$FIXTURE" commit -qm 'Touch EKF' \
        -m 'Algorithm-Identity-Unchanged: claimed early' \
        -m 'More prose after the would-be trailer block.'
    expect fail 'trailer embedded mid-body is not a trailer'

    fixture_branch s-trailer-fake
    append_line aviate-core/src/ekf/update.rs '// probe'
    git -C "$FIXTURE" add -A
    git -C "$FIXTURE" commit -qm 'Touch EKF' \
        -m 'X-Algorithm-Identity-Unchanged: look-alike key'
    expect fail 'look-alike trailer key is rejected'

    fixture_branch s-trailer-dup
    append_line aviate-core/src/ekf/update.rs '// probe'
    git -C "$FIXTURE" add -A
    git -C "$FIXTURE" commit -qm 'Touch EKF' \
        -m $'Algorithm-Identity-Unchanged: first claim\nAlgorithm-Identity-Unchanged: second claim'
    expect fail 'duplicate trailers are rejected'

    fixture_branch s-trailer-empty
    append_line aviate-core/src/ekf/update.rs '// probe'
    git -C "$FIXTURE" add -A
    git -C "$FIXTURE" commit -qm 'Touch EKF' -m 'Algorithm-Identity-Unchanged:'
    expect fail 'bare trailer without rationale is rejected'

    # Direct-push ranges: several commits between before and sha, only
    # some of which touch production code.
    fixture_branch s-push-range-bad
    append_line aviate-core/src/ekf/update.rs '// probe'
    fixture_commit 'unadjudicated EKF change'
    append_line aviate-link/src/queue.rs '// innocent follow-up'
    fixture_commit 'innocent non-production commit'
    expect fail 'multi-commit push range with one unadjudicated commit fails'

    fixture_branch s-push-range-good
    append_line aviate-core/src/ekf/update.rs '// probe'
    git -C "$FIXTURE" add -A
    git -C "$FIXTURE" commit -qm 'Touch EKF comment' \
        -m 'Algorithm-Identity-Unchanged: comment-only change, no executable difference'
    append_line aviate-link/src/queue.rs '// innocent follow-up'
    fixture_commit 'innocent non-production commit'
    expect pass 'multi-commit push range where every production commit is adjudicated'

    fixture_cleanup
    trap - EXIT

    if [[ $SELF_TEST_FAILURES -ne 0 ]]; then
        echo "Identity-adjudication self-test: $SELF_TEST_FAILURES scenario(s) FAILED" >&2
        return 1
    fi
    echo "Identity-adjudication self-test: OK"
}

# ------------------------------------------------------------- dispatch

case "${1:-}" in
    --self-test)
        self_test
        ;;
    --structural)
        rev="${2:?usage: $0 --structural <rev>}"
        structural_check "$rev" && echo "Ledger coherent at $rev."
        ;;
    "")
        echo "usage: $0 <base-rev> <head-rev> | --structural <rev> | --self-test" >&2
        exit 2
        ;;
    *)
        base="$1"
        head="${2:?usage: $0 <base-rev> <head-rev>}"
        adjudicate "$base" "$head"
        ;;
esac
