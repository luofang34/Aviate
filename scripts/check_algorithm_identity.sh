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
# without either rotating the identity of EVERY implementation that
# owns the changed file in cert/algorithm_id_registry.toml — each to
# an ID never before present anywhere in the ledger — or a human
# stating, per commit, as an exact git trailer, why behavior cannot
# have changed:
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
#   * every production implementation of an adjudicated trait has
#     exactly one IMPL_MAP row (reverse coverage — a new impl under a
#     covered root cannot escape adjudication, even by aliasing an
#     already-registered ID);
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
#   scripts/check_algorithm_identity.sh --pr-range <base-rev> <head-rev>
#   scripts/check_algorithm_identity.sh --push-range <before> <sha> [main-ref]
#   scripts/check_algorithm_identity.sh --structural <rev>
#   scripts/check_algorithm_identity.sh --self-test
#
# --pr-range adds the squash-survivability guard: a trailer-reliant
# multi-commit PR must carry the trailer on its final commit, so the
# commit-message-composed squash body still ends in a parseable
# trailer block when it reaches the push gate on main.
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
# A file owned by several entries (shared code) is rotation-
# adjudicated only when EVERY owner rotates: rotating one sibling
# would otherwise mask a change to another sibling sharing the file.
# An entry that does not own the file never satisfies the gate for
# it. The sanitizer tree is explicit so a sanitizer change is
# adjudicated against the sanitizer identity, not waved through as
# generic mixer-tree churn.
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

    if ! check_impl_coverage "$rev"; then
        bad=1
    fi

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

# Reverse IMPL_MAP completeness: every production implementation of
# an adjudicated trait — an impl block under a production root that
# declares an ALGORITHM_ID constant — must have exactly one IMPL_MAP
# row for its (file, type). Without this, a new implementation added
# under a covered root would carry an identity the gate never
# adjudicates (worst case: aliasing an already-registered ID, which
# the literal-registration check alone cannot see).
check_impl_coverage() {
    local rev="$1" bad=0
    local hit file type row msec mkey mtype mfile matches
    while IFS= read -r hit; do
        [[ -z "$hit" ]] && continue
        file="${hit#"$rev":}"
        is_test_path "$file" && continue
        [[ -z "$(owners_for "$file")" ]] && continue
        while IFS= read -r type; do
            [[ -z "$type" ]] && continue
            matches=0
            for row in "${IMPL_MAP[@]}"; do
                IFS='|' read -r msec mkey mtype mfile <<< "$row"
                if [[ "$mfile" == "$file" && "$mtype" == "$type" ]]; then
                    matches=$(( matches + 1 ))
                fi
            done
            if [[ $matches -eq 0 ]]; then
                echo "FAIL: production implementation $type in $file has no IMPL_MAP row; its identity is never adjudicated" >&2
                bad=1
            elif [[ $matches -gt 1 ]]; then
                echo "FAIL: $matches IMPL_MAP rows claim $type in $file; ownership must be unambiguous" >&2
                bad=1
            fi
        done < <(git -C "$REPO_ROOT" show "$rev:$file" 2>/dev/null | awk '
            /^impl/ {
                type = ""
                if (match($0, / for [A-Za-z0-9_:]+/)) {
                    type = substr($0, RSTART + 5, RLENGTH - 5)
                    sub(/.*::/, "", type)
                }
            }
            /^\}/ { type = "" }
            type != "" && /const[ \t]+ALGORITHM_ID[ \t]*:[ \t]*u64[ \t]*=[ \t]*0x/ {
                print type
                type = ""
            }
        ')
    done < <(git -C "$REPO_ROOT" grep -l -E 'const[ \t]+ALGORITHM_ID[ \t]*:[ \t]*u64[ \t]*=[ \t]*0x' "$rev" -- '*.rs' 2>/dev/null || true)
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

# adjudicate <base> <head> [guard]
# guard 'squash' adds the PR-only squash-survivability requirement:
# when trailer adjudication is in play and the range has more than
# one commit, the final commit must itself carry the trailer. The
# repository composes squash-merge messages from the commit messages,
# so the final commit's trailer block is what ends the squash body —
# it is the only part of the composition that the push gate on main
# will still parse as a final trailer.
adjudicate() {
    local base="$1" head="$2" guard="${3:-}" bad=0

    structural_check "$head" || bad=1
    continuity_check "$base" "$head" || bad=1

    # --no-renames: a rename out of a managed tree must surface the
    # old managed path as a deletion and be adjudicated, not be
    # collapsed into an unmanaged new path by rename detection.
    local changed
    changed="$(git -C "$REPO_ROOT" diff --no-renames --name-only "$base" "$head")"

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

    # Every numeric ID present anywhere in the base ledger — active in
    # any section, or retired. A rotation must mint an ID outside this
    # set: renaming a key while keeping its number, or two entries
    # swapping numbers, changes nothing at the lockstep gate and must
    # not count as a rotation.
    local base_ids
    base_ids="$(awk '{ print ($1 == "A") ? $4 : $2 }' <<< "$base_dump" | sort -u)"

    # A changed file is rotation-adjudicated only when EVERY identity
    # that owns it rotated to a genuinely new ID: rotating one sibling
    # of a shared file must not mask a change to another sibling.
    # Files with any unrotated owner fall through to the per-commit
    # trailer requirement.
    local touched_files unadjudicated='' unrot_report=''
    touched_files="$(cut -d'|' -f2 <<< "$pairs" | sort -u)"
    while IFS= read -r file; do
        [[ -z "$file" ]] && continue
        local unrotated='' owner section key head_val
        while IFS= read -r owner; do
            [[ -z "$owner" ]] && continue
            section="${owner%%/*}"
            key="${owner#*/}"
            head_val="$(registry_value "$head_dump" "$section" "$key")"
            if [[ -n "$head_val" ]] && ! grep -qxF "$head_val" <<< "$base_ids"; then
                echo "Identity rotated for $file: [$section] \"$key\" holds an ID new to the ledger."
            else
                unrotated+="${unrotated:+, }$owner"
            fi
        done < <(awk -F'|' -v f="$file" '$2 == f { print $1 }' <<< "$pairs" | sort -u)
        if [[ -n "$unrotated" ]]; then
            unadjudicated+="$file"$'\n'
            unrot_report+="  $file (unrotated: $unrotated)"$'\n'
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
        commits="$(git -C "$REPO_ROOT" log --no-renames --format=%H "$base..$head" -- "${paths[@]}" 2>/dev/null || true)"
        if [[ -z "$commits" ]]; then
            echo "FAIL: production files changed in $base..$head but no commit claims them (history rewrite?); cannot adjudicate:" >&2
            printf '%s' "$unrot_report" >&2
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
                printf '%s' "$unrot_report" >&2
                echo "and these commits touch them without a valid '$TRAILER_KEY' trailer" >&2
                echo "(exact final trailer, exactly once, non-empty rationale):" >&2
                printf '%s' "$missing" | sed 's/^/  /' >&2
                echo "Either rotate every owning identity in $REGISTRY (to IDs new to the ledger) or add, per commit:" >&2
                echo "  $TRAILER_KEY: <why this cannot change observable behavior>" >&2
                bad=1
            else
                echo "Identity adjudicated: every commit touching non-rotated production files carries an exact $TRAILER_KEY trailer."
                if [[ "$guard" == 'squash' ]]; then
                    local range_commits
                    range_commits="$(git -C "$REPO_ROOT" rev-list --count "$base..$head")"
                    if [[ "$range_commits" -gt 1 ]] && ! trailer_ok "$head"; then
                        echo "FAIL: this range relies on $TRAILER_KEY trailers but its final commit does not carry one." >&2
                        echo "A squash merge composes its message from the commit messages, so only the final" >&2
                        echo "commit's trailer block survives as the squash commit's final trailer; without it" >&2
                        echo "the push gate rejects the squashed commit after it lands on main." >&2
                        echo "Add the trailer to the final commit, rotate the identity instead, or merge without squashing." >&2
                        bad=1
                    fi
                fi
            fi
        fi
    fi

    if [[ $bad -eq 0 ]]; then
        echo "Identity adjudication passed for $base..$head."
    fi
    return "$bad"
}

# ------------------------------------------------------- push ranges

# Resolve a push event's before/sha pair into an adjudication,
# fail-closed. Only two shapes may skip range adjudication: a
# genuinely empty push (before == sha) and the creation of a branch
# whose head is already main history — both still get the structural
# ledger check. A nonzero before that this checkout cannot resolve
# means a force-push or truncated fetch: refusing is the point, since
# a silent structural-only fallback would wave exactly the
# direct-push case this gate exists for.
push_range() {
    local before="$1" sha="$2" main_ref="${3:-origin/main}"
    local zero='0000000000000000000000000000000000000000'

    if [[ "$before" == "$sha" ]]; then
        echo "Empty push (before == sha); checking ledger coherence only."
        structural_check "$sha" && echo "Ledger coherent at $sha."
        return
    fi

    if [[ -z "$before" || "$before" == "$zero" ]]; then
        # Branch creation: no prior tip exists, so adjudicate what the
        # branch adds over main.
        local mb head_commit
        mb="$(git -C "$REPO_ROOT" merge-base "$main_ref" "$sha" 2>/dev/null || true)"
        if [[ -z "$mb" ]]; then
            echo "FAIL: cannot establish a push range: no merge base between $main_ref and $sha (disjoint history); refusing to adjudicate" >&2
            return 1
        fi
        head_commit="$(git -C "$REPO_ROOT" rev-parse "$sha^{commit}")"
        if [[ "$mb" == "$head_commit" ]]; then
            echo "Branch created at existing $main_ref history; checking ledger coherence only."
            structural_check "$sha" && echo "Ledger coherent at $sha."
            return
        fi
        adjudicate "$mb" "$sha"
        return
    fi

    if ! git -C "$REPO_ROOT" rev-parse --quiet --verify "$before^{commit}" > /dev/null 2>&1; then
        echo "FAIL: push before-SHA $before is unreachable in this checkout (force-push or shallow/truncated fetch history); refusing to adjudicate a partial range" >&2
        return 1
    fi

    adjudicate "$before" "$sha"
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
        aviate-core/src/control/attitude.rs \
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

# Shared verdict checker for the expect_* helpers below: compares the
# observed pass/fail against the expectation and, when a pattern is
# given, requires the output to contain it — so a scenario cannot
# "pass" by failing for an unrelated reason.
SELF_TEST_FAILURES=0
check_verdict() {
    local want="$1" name="$2" pattern="$3" got="$4" out="$5"
    if [[ "$got" != "$want" ]]; then
        echo "SELF-TEST FAIL: $name — expected $want, got $got" >&2
        sed 's/^/    /' <<< "$out" >&2
        SELF_TEST_FAILURES=$(( SELF_TEST_FAILURES + 1 ))
        return
    fi
    if [[ -n "$pattern" ]] && ! grep -q "$pattern" <<< "$out"; then
        echo "SELF-TEST FAIL: $name — verdict $got but output lacks '$pattern'" >&2
        sed 's/^/    /' <<< "$out" >&2
        SELF_TEST_FAILURES=$(( SELF_TEST_FAILURES + 1 ))
        return
    fi
    echo "  ok: $name ($want)"
}

# expect <pass|fail> <name> — adjudicate main..HEAD on the fixture.
expect() {
    expect_msg "$1" '' "$2"
}

# expect_msg <pass|fail> <grep-pattern> <name> — same, and the output
# must contain the pattern.
expect_msg() {
    local want="$1" pattern="$2" name="$3" out got
    if out="$( (REPO_ROOT="$FIXTURE"; adjudicate main HEAD) 2>&1 )"; then
        got='pass'
    else
        got='fail'
    fi
    check_verdict "$want" "$name" "$pattern" "$got" "$out"
}

# expect_push <pass|fail> <before> <sha> <grep-pattern> <name> —
# drives the push-range resolution against the fixture, with the
# fixture's own main as the protected ref.
expect_push() {
    local want="$1" before="$2" sha="$3" pattern="$4" name="$5" out got
    if out="$( (REPO_ROOT="$FIXTURE"; push_range "$before" "$sha" main) 2>&1 )"; then
        got='pass'
    else
        got='fail'
    fi
    check_verdict "$want" "$name" "$pattern" "$got" "$out"
}

# expect_pr <pass|fail> <grep-pattern> <name> — adjudicates
# main..HEAD with the PR-only squash-survivability guard.
expect_pr() {
    local want="$1" pattern="$2" name="$3" out got
    if out="$( (REPO_ROOT="$FIXTURE"; adjudicate main HEAD squash) 2>&1 )"; then
        got='pass'
    else
        got='fail'
    fi
    check_verdict "$want" "$name" "$pattern" "$got" "$out"
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

    # Shared file, wrong sibling: attitude.rs is owned by all three
    # controllers; rotating only vtol must not adjudicate it.
    fixture_branch s-wrong-sibling
    append_line aviate-core/src/control/attitude.rs '// probe'
    replace_in "$REGISTRY" '544F_4C31' '544F_4C32'
    replace_in aviate-core/src/control/vtol.rs '544F_4C31' '544F_4C32'
    append_line "$REGISTRY" '# Retired: "controller.vtol.v1" = 0x4354_4C56_544F_4C31.'
    fixture_commit 'shared control change hidden behind a single sibling rotation'
    expect_msg fail 'attitude.rs (unrotated: .*controller/controller.multirotor.v2' \
        'shared-file change requires every owning sibling to rotate'

    # Key rename keeping the numeric ID: no lockstep-visible rotation
    # happened, so the gate must not treat it as one.
    fixture_branch s-rename-launder
    append_line aviate-core/src/control/vtol.rs '// probe'
    replace_in "$REGISTRY" '"controller.vtol.v1"' '"controller.vtol.v9"'
    fixture_commit 'key renamed, numeric ID kept'
    expect_msg fail 'unrotated: controller/controller.vtol.v1' \
        'key rename keeping the old numeric ID is not a rotation'

    # Two entries swapping numeric IDs: parity, uniqueness, reuse and
    # continuity all still hold — only the IDs-new-to-the-ledger rule
    # can catch it.
    fixture_branch s-id-swap
    replace_in "$REGISTRY" '4354_4C56_544F_4C31' 'SWAP_PLACEHOLDER'
    replace_in "$REGISTRY" '4354_4C46_5747_5631' '4354_4C56_544F_4C31'
    replace_in "$REGISTRY" 'SWAP_PLACEHOLDER' '4354_4C46_5747_5631'
    replace_in aviate-core/src/control/vtol.rs '4354_4C56_544F_4C31' '4354_4C46_5747_5631'
    replace_in aviate-core/src/control/fixed_wing.rs '4354_4C46_5747_5631' '4354_4C56_544F_4C31'
    fixture_commit 'vtol and fixed_wing swap numeric IDs'
    expect_msg fail 'unrotated: controller/controller.vtol.v1' \
        'two entries swapping numeric IDs is not a rotation'

    # Rename escape: moving a managed implementation out of its tree
    # must adjudicate the old managed path, not vanish behind rename
    # detection.
    fixture_branch s-rename-escape
    mkdir -p "$FIXTURE/aviate-link/src"
    git -C "$FIXTURE" mv aviate-core/src/control/vtol.rs aviate-link/src/vtol_moved.rs
    append_line aviate-link/src/vtol_moved.rs '// tweak after the move'
    fixture_commit 'vtol implementation renamed out of the managed tree'
    expect_msg fail 'aviate-core/src/control/vtol.rs (unrotated' \
        'rename out of a managed tree is adjudicated at the old path'

    # Reverse coverage: an implementation added under a covered root
    # that aliases an already-registered ID passes every literal and
    # parity check — only the impl-to-IMPL_MAP enumeration can see it.
    fixture_branch s-unmapped-impl
    cat > "$FIXTURE/aviate-core/src/ekf/experimental.rs" <<'RS'
//! Experimental estimator variant.

impl super::Estimator for Ekf2 {
    const ALGORITHM_ID: u64 = 0x4554_494D_454B_4634; // aliases Ekf
}
RS
    fixture_commit 'new estimator impl without an IMPL_MAP row'
    expect_msg fail 'Ekf2 in aviate-core/src/ekf/experimental.rs has no IMPL_MAP row' \
        'unmapped production implementation is rejected by reverse coverage'

    # Squash survivability: a trailer-reliant multi-commit PR whose
    # final commit lacks the trailer would land on main as a squash
    # commit the push gate rejects — catch it before merge.
    fixture_branch s-squash-buried
    append_line aviate-core/src/ekf/update.rs '// probe'
    git -C "$FIXTURE" add -A
    git -C "$FIXTURE" commit -qm 'Touch EKF comment' \
        -m 'Algorithm-Identity-Unchanged: comment-only change, no executable difference'
    append_line aviate-link/src/queue.rs '// innocent follow-up'
    fixture_commit 'innocent non-production commit'
    expect_pr fail 'final commit does not carry one' \
        'trailer-reliant PR whose final commit lacks the trailer fails the squash guard'

    fixture_branch s-squash-final
    append_line aviate-core/src/ekf/update.rs '// probe'
    git -C "$FIXTURE" add -A
    git -C "$FIXTURE" commit -qm 'Touch EKF comment' \
        -m 'Algorithm-Identity-Unchanged: comment-only change, no executable difference'
    append_line aviate-link/src/queue.rs '// innocent follow-up'
    git -C "$FIXTURE" add -A
    git -C "$FIXTURE" commit -qm 'Innocent follow-up' \
        -m 'Algorithm-Identity-Unchanged: restates the range claim so the squash body ends with it'
    expect_pr pass '' 'trailer on the final commit satisfies the squash guard'

    fixture_branch s-squash-single
    append_line aviate-core/src/ekf/update.rs '// probe'
    git -C "$FIXTURE" add -A
    git -C "$FIXTURE" commit -qm 'Touch EKF comment' \
        -m 'Algorithm-Identity-Unchanged: comment-only change, no executable difference'
    expect_pr pass '' 'single-commit trailer PR passes the squash guard'

    fixture_branch s-squash-rotation
    replace_in "$REGISTRY" '544F_4C31' '544F_4C32'
    replace_in aviate-core/src/control/vtol.rs '544F_4C31' '544F_4C32'
    append_line "$REGISTRY" '# Retired: "controller.vtol.v1" = 0x4354_4C56_544F_4C31.'
    fixture_commit 'vtol behavior change with its own rotation'
    append_line aviate-link/src/queue.rs '// innocent follow-up'
    fixture_commit 'innocent non-production commit'
    expect_pr pass '' 'rotation-adjudicated multi-commit PR needs no squash trailer'

    # Push-range resolution: fail-closed on anything that is not a
    # provable range, structural-only for the two honest no-range
    # shapes.
    local zero='0000000000000000000000000000000000000000'
    local main_sha
    main_sha="$(git -C "$FIXTURE" rev-parse main)"

    fixture_branch p-ahead
    append_line aviate-core/src/ekf/update.rs '// probe'
    fixture_commit 'unadjudicated EKF change'
    local p_ahead
    p_ahead="$(git -C "$FIXTURE" rev-parse HEAD)"

    fixture_branch p-old
    append_line aviate-link/src/queue.rs '// pre-rewrite tip'
    fixture_commit 'non-production commit on the old tip'
    local p_old
    p_old="$(git -C "$FIXTURE" rev-parse HEAD)"

    expect_push fail 'deadbeefdeadbeefdeadbeefdeadbeefdeadbeef' "$p_ahead" \
        'unreachable' 'unreachable push before-SHA fails closed'
    expect_push fail "$p_old" "$p_ahead" 'without identity rotation' \
        'force-push rewrite range is adjudicated, not waved through'
    expect_push pass "$main_sha" "$main_sha" 'Ledger coherent' \
        'genuinely empty push runs the structural check only'
    expect_push pass "$zero" "$main_sha" 'Ledger coherent' \
        'branch creation at existing main history is structural-only'
    expect_push fail "$zero" "$p_ahead" 'without identity rotation' \
        'branch creation adjudicates its new commits against main'

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
    --pr-range)
        base="${2:?usage: $0 --pr-range <base-rev> <head-rev>}"
        head="${3:?usage: $0 --pr-range <base-rev> <head-rev>}"
        adjudicate "$base" "$head" squash
        ;;
    --push-range)
        before="${2:?usage: $0 --push-range <before-sha> <sha> [main-ref]}"
        sha="${3:?usage: $0 --push-range <before-sha> <sha> [main-ref]}"
        push_range "$before" "$sha" "${4:-origin/main}"
        ;;
    "")
        echo "usage: $0 <base-rev> <head-rev> | --pr-range <base> <head> | --push-range <before> <sha> [main-ref] | --structural <rev> | --self-test" >&2
        exit 2
        ;;
    *)
        base="$1"
        head="${2:?usage: $0 <base-rev> <head-rev>}"
        adjudicate "$base" "$head"
        ;;
esac
