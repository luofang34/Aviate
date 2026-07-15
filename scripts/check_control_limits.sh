#!/usr/bin/env bash
#
# Control-limit classification gate.
#
# Every behavior-shaping numeric limit in the control law must be one
# of two things, and nothing else:
#
#   * tuning     — a `CascadeGains` field: validated at construction,
#                  fed into `ResolvedKernelConfig::canonical_hash` and
#                  `controller_tuning_identity`, and probed by both
#                  per-field mutation sweeps;
#   * invariant  — a named `Scalar` constant with a WHY rationale,
#                  registered with its pinned value, living inside the
#                  controller-owned tree so the algorithm-identity gate
#                  adjudicates any change.
#
# The registry is cert/control_limits_registry.toml. This script makes
# the classification executable:
#
#   1. [[tuning]] entries and `CascadeGains` fields are a bijection;
#      every field is fed by `feed_cascade_gains`, probed by the
#      canonical-hash mutation sweep, and mutated by the builder
#      binding sweep.
#   2. Every [[invariant]] entry is declared in its registered file
#      with exactly its registered value.
#   3. Every `const NAME: Scalar` declaration in a scanned file is a
#      registered invariant (aliases of `core::f32::consts::*` exempt),
#      so an unregistered constant fails even when its value happens
#      to be structural algebra.
#   4. Every float literal in a scanned production line is structural
#      algebra (0.0 / 1.0 / 2.0 / 0.5), a registered tuning field's
#      default in cascade_gains.rs, or a registered invariant
#      declaration.
#
# Scan boundary: aviate-core/src/control.rs plus production files under
# aviate-core/src/control/ (test modules and *tests.rs excluded; float
# literals only — `Scalar` is the type of every control-law knob).
# Constants imported from outside the boundary reach a controller only
# through a reviewed cross-module import, which this gate does not
# claim to cover.
#
# Usage:
#   scripts/check_control_limits.sh              # check this checkout
#   scripts/check_control_limits.sh --self-test  # adversarial fixtures
#
# Exit codes: 0 coherent, 1 classification failure, 2 bad invocation.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# CTLLIM_REPO_ROOT points the checks at another checkout (the
# self-test fixture); CI never sets it.
REPO_ROOT="${CTLLIM_REPO_ROOT:-$(cd "$SCRIPT_DIR/.." && pwd)}"

REGISTRY='cert/control_limits_registry.toml'
GAINS_FILE='aviate-core/src/control/cascade_gains.rs'
CANONICAL_FILE='aviate-core/src/kernel/config/canonical.rs'
CANONICAL_TESTS='aviate-core/src/kernel/config/canonical/tests.rs'
BINDING_TESTS='aviate-core/tests/config_binding_tests.rs'
CONTROL_ROOT='aviate-core/src/control'

# Structural algebra: identities, sign flips, and exact binary
# doubling/halving (quaternion axis-angle doubling, midpoints, unit
# bounds). Everything else is behavior-shaping and must be registered.
ALLOWED_FLOATS='0.0 1.0 2.0 0.5'

# ------------------------------------------------------ registry parse

registry_tuning() {
    awk '
        /^\[\[tuning\]\]/ { sect = "t"; next }
        /^\[\[/           { sect = "";  next }
        sect == "t" && /^field[ \t]*=/ && match($0, /"[^"]*"/) {
            print substr($0, RSTART + 1, RLENGTH - 2)
        }
    ' "$REPO_ROOT/$REGISTRY"
}

# Emits one entry per line: name<TAB>file<TAB>value
registry_invariants() {
    awk '
        /^\[\[invariant\]\]/ { sect = "i"; name = ""; file = ""; next }
        /^\[\[/              { sect = "";  next }
        sect == "i" && /^name[ \t]*=/ && match($0, /"[^"]*"/) {
            name = substr($0, RSTART + 1, RLENGTH - 2); next
        }
        sect == "i" && /^file[ \t]*=/ && match($0, /"[^"]*"/) {
            file = substr($0, RSTART + 1, RLENGTH - 2); next
        }
        sect == "i" && /^value[ \t]*=/ && match($0, /"[^"]*"/) {
            printf "%s\t%s\t%s\n", name, file, substr($0, RSTART + 1, RLENGTH - 2)
        }
    ' "$REPO_ROOT/$REGISTRY"
}

# ------------------------------------------------------------ scanning

scanned_files() {
    printf '%s\n' 'aviate-core/src/control.rs'
    (cd "$REPO_ROOT" && find "$CONTROL_ROOT" -name '*.rs' ! -name '*tests.rs' | sort)
}

# Emit `lineno<TAB>code` for the production lines of a file: string
# literals blanked, comments stripped, `#[cfg(test)]` items dropped
# (both `mod tests;` re-declarations and inline `mod tests { … }`
# blocks — relies on rustfmt closing top-level blocks at column 0).
production_lines() {
    awk '
        /^#\[cfg\(test\)\]/ { skip_attr = 1; next }
        skip_attr && /^#\[/ { next }
        skip_attr && /;[ \t]*$/ { skip_attr = 0; next }
        skip_attr && /\{[ \t]*$/ { skip_attr = 0; in_test = 1; next }
        in_test && /^\}/ { in_test = 0; next }
        in_test { next }
        {
            line = $0
            gsub(/"[^"]*"/, "\"\"", line)
            sub(/\/\/.*/, "", line)
            printf "%d\t%s\n", NR, line
        }
    ' "$REPO_ROOT/$1"
}

# --------------------------------------------------------- the checks

# [[tuning]] ↔ CascadeGains bijection, plus per-field hash feed and
# both mutation sweeps.
check_tuning() {
    local bad=0
    local struct_fields reg_fields
    struct_fields="$(awk '
        /^pub struct CascadeGains \{/ { s = 1; next }
        s && /^\}/ { exit }
        s && /^    pub [a-z][a-z0-9_]*:/ {
            f = $2; sub(/:.*/, "", f); print f
        }
    ' "$REPO_ROOT/$GAINS_FILE" | sort)"
    reg_fields="$(registry_tuning | sort)"

    local only_struct only_reg
    only_struct="$(comm -23 <(printf '%s\n' "$struct_fields") <(printf '%s\n' "$reg_fields"))"
    only_reg="$(comm -13 <(printf '%s\n' "$struct_fields") <(printf '%s\n' "$reg_fields"))"
    if [[ -n "$only_struct" ]]; then
        echo "FAIL: CascadeGains fields missing from the [[tuning]] registry ($REGISTRY):" >&2
        sed 's/^/  /' <<< "$only_struct" >&2
        bad=1
    fi
    if [[ -n "$only_reg" ]]; then
        echo "FAIL: [[tuning]] registry names fields that CascadeGains does not declare:" >&2
        sed 's/^/  /' <<< "$only_reg" >&2
        bad=1
    fi

    local f
    while IFS= read -r f; do
        [[ -z "$f" ]] && continue
        # `.field` rather than `g.field`: rustfmt may break the
        # receiver onto its own line in a method chain.
        if ! grep -qE "\.$f([^a-z0-9_]|$)" "$REPO_ROOT/$CANONICAL_FILE"; then
            echo "FAIL: tuning field '$f' is not fed into the canonical hash ($CANONICAL_FILE)" >&2
            bad=1
        fi
        if ! grep -qE "probe!\($f[,)]" "$REPO_ROOT/$CANONICAL_TESTS"; then
            echo "FAIL: tuning field '$f' has no canonical-hash mutation probe ($CANONICAL_TESTS)" >&2
            bad=1
        fi
        if ! grep -qE "cascade_gains\.$f([^a-z0-9_]|$)" "$REPO_ROOT/$BINDING_TESTS"; then
            echo "FAIL: tuning field '$f' has no builder binding-sweep mutation ($BINDING_TESTS)" >&2
            bad=1
        fi
    done <<< "$reg_fields"
    return "$bad"
}

# Every registered invariant is declared, in its registered file, with
# exactly its registered value.
check_invariants() {
    local bad=0 name file value
    while IFS=$'\t' read -r name file value; do
        [[ -z "$name" ]] && continue
        if [[ ! -f "$REPO_ROOT/$file" ]]; then
            echo "FAIL: [[invariant]] $name names a missing file: $file" >&2
            bad=1
            continue
        fi
        if ! grep -qF "const $name: Scalar = $value;" "$REPO_ROOT/$file"; then
            echo "FAIL: [[invariant]] $name is not declared in $file with its registered value '$value'" >&2
            bad=1
        fi
    done < <(registry_invariants)
    return "$bad"
}

# Literal scan: rules 3 and 4 of the header.
check_literals() {
    local bad=0 file lineno code
    local tuning_fields invariants
    tuning_fields="$(registry_tuning)"
    invariants="$(registry_invariants)"

    while IFS= read -r file; do
        [[ -f "$REPO_ROOT/$file" ]] || continue
        while IFS=$'\t' read -r lineno code; do
            [[ -z "$code" ]] && continue

            # Compile-time bounds cannot shape flight behavior (a
            # violated one is a build error, not a different binary).
            if [[ "$code" =~ ^const[[:space:]]+_:[[:space:]]*\(\)[[:space:]]*=[[:space:]]*assert! ]]; then
                continue
            fi

            # Scalar constant declarations must be registered.
            if [[ "$code" =~ const[[:space:]]+([A-Z][A-Z0-9_]*)[[:space:]]*:[[:space:]]*Scalar ]]; then
                local cname="${BASH_REMATCH[1]}"
                if [[ "$code" == *'= core::f32::consts::'* ]]; then
                    continue # alias of a math constant, not a limit
                fi
                local entry
                entry="$(awk -F'\t' -v n="$cname" -v f="$file" '$1 == n && $2 == f' <<< "$invariants")"
                if [[ -z "$entry" ]]; then
                    echo "FAIL: $file:$lineno: const $cname: Scalar is not registered as an [[invariant]] in $REGISTRY" >&2
                    bad=1
                elif [[ "$code" != *"= $(cut -f3 <<< "$entry");"* ]]; then
                    echo "FAIL: $file:$lineno: const $cname declares a value different from its registered '$(cut -f3 <<< "$entry")'" >&2
                    bad=1
                fi
                continue
            fi

            local floats
            floats="$(grep -oE '[0-9]+\.[0-9]+([eE][+-]?[0-9]+)?|[0-9]+[eE][+-]?[0-9]+' <<< "$code" || true)"
            [[ -z "$floats" ]] && continue

            # cascade_gains.rs: literals are legitimate as defaults of
            # registered tuning fields (the hash-covered surface).
            if [[ "$file" == "$GAINS_FILE" ]]; then
                local fld matched=0
                while IFS= read -r fld; do
                    [[ -z "$fld" ]] && continue
                    if [[ "$code" =~ ^[[:space:]]*"$fld":[[:space:]] ]]; then
                        matched=1
                        break
                    fi
                done <<< "$tuning_fields"
                [[ $matched -eq 1 ]] && continue
            fi

            local tok
            while IFS= read -r tok; do
                case " $ALLOWED_FLOATS " in
                    *" $tok "*) ;;
                    *)
                        echo "FAIL: $file:$lineno: unregistered float literal $tok in a control-law implementation body." >&2
                        echo "  Classify it: a CascadeGains tuning field (validated + hash-covered + mutation-swept)" >&2
                        echo "  or a registered invariant constant with a WHY rationale (see $REGISTRY)." >&2
                        bad=1
                        ;;
                esac
            done <<< "$floats"
        done < <(production_lines "$file")
    done < <(scanned_files)
    return "$bad"
}

run_checks() {
    local bad=0
    if [[ ! -f "$REPO_ROOT/$REGISTRY" ]]; then
        echo "FAIL: missing $REGISTRY" >&2
        return 1
    fi
    check_tuning || bad=1
    check_invariants || bad=1
    check_literals || bad=1
    if [[ $bad -eq 0 ]]; then
        echo "Control-limit classification is coherent: every scanned literal is structural, hash-covered tuning, or a registered invariant."
    fi
    return "$bad"
}

# ------------------------------------------------------------ self-test

FIXTURE=''

fixture_cleanup() {
    [[ -n "$FIXTURE" && -d "$FIXTURE" ]] && rm -rf "$FIXTURE"
    return 0
}

# Fresh copy of the real registry + real sources per scenario, so
# every scenario mutates truth, not a synthetic stub.
fixture_reset() {
    fixture_cleanup
    FIXTURE="$(mktemp -d "${TMPDIR:-/tmp}/ctllim-selftest.XXXXXX")"
    (cd "$REPO_ROOT" && tar -cf - \
        "$REGISTRY" \
        "$CANONICAL_FILE" \
        "$CANONICAL_TESTS" \
        "$BINDING_TESTS" \
        aviate-core/src/control.rs \
        "$CONTROL_ROOT") | (cd "$FIXTURE" && tar -xf -)
}

prepend_line() {
    local f="$FIXTURE/$1" text="$2" tmp
    tmp="$(mktemp)"
    printf '%s\n' "$text" | cat - "$f" > "$tmp"
    mv "$tmp" "$f"
}

append_line() {
    printf '%s\n' "$2" >> "$FIXTURE/$1"
}

# perl for portable in-place replacement with \n in the replacement.
replace_in() {
    perl -0pi -e "s/\Q$2\E/$3/" "$FIXTURE/$1"
}

SELF_TEST_FAILURES=0
expect() {
    local want="$1" pattern="$2" name="$3" out got
    if out="$( (REPO_ROOT="$FIXTURE"; run_checks) 2>&1 )"; then
        got='pass'
    else
        got='fail'
    fi
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

self_test() {
    trap fixture_cleanup EXIT

    fixture_reset
    expect pass 'classification is coherent' \
        'pristine tree passes'

    # The issue-mandated case: a synthetic unregistered constant.
    fixture_reset
    prepend_line "$CONTROL_ROOT/velocity.rs" 'const SNEAKY_GAIN: Scalar = 4.2;'
    expect fail 'not registered as an \[\[invariant\]\]' \
        'synthetic unregistered constant fails'

    fixture_reset
    prepend_line "$CONTROL_ROOT/rate.rs" 'let sneaky = p_error * 0.037;'
    expect fail 'unregistered float literal 0.037' \
        'bare behavior-shaping literal in an impl body fails'

    fixture_reset
    prepend_line "$CONTROL_ROOT/attitude.rs" 'const SNEAKY_HALF: Scalar = 0.5;'
    expect fail 'not registered as an \[\[invariant\]\]' \
        'unregistered constant fails even with an allowlisted value'

    fixture_reset
    replace_in "$GAINS_FILE" 'pub struct CascadeGains {' \
        'pub struct CascadeGains {\n    pub sneaky_knob: Scalar,'
    expect fail 'missing from the \[\[tuning\]\] registry' \
        'CascadeGains field without a registry entry fails'

    fixture_reset
    append_line "$REGISTRY" '[[tuning]]'
    append_line "$REGISTRY" 'field = "ghost_knob"'
    expect fail 'does not declare' \
        'registry tuning entry without a struct field fails'

    fixture_reset
    replace_in "$CONTROL_ROOT/law_invariants.rs" \
        'DISARMED_COLLECTIVE_THRESHOLD: Scalar = 0.02;' \
        'DISARMED_COLLECTIVE_THRESHOLD: Scalar = 0.03;'
    expect fail 'registered' \
        'invariant value drift without a registry edit fails'

    fixture_reset
    replace_in "$REGISTRY" 'name = "TILT_COMP_COS_FLOOR"' \
        'name = "TILT_COMP_COS_FLOOR_GONE"'
    expect fail 'not declared\|not registered' \
        'dropping an invariant registry entry fails both directions'

    fixture_reset
    replace_in "$CANONICAL_TESTS" '    probe!(vel_max_yaw_step, 0.05);' ''
    expect fail 'no canonical-hash mutation probe' \
        'removing a canonical-sweep probe fails'

    fixture_reset
    replace_in "$BINDING_TESTS" 'cascade_gains.att_max_rate_cmd += 0.125' \
        'cascade_gains.att_p[1] += 0.125'
    expect fail 'no builder binding-sweep mutation' \
        'removing a binding-sweep mutation fails'

    fixture_cleanup
    trap - EXIT

    if [[ $SELF_TEST_FAILURES -ne 0 ]]; then
        echo "Control-limit gate self-test: $SELF_TEST_FAILURES scenario(s) FAILED" >&2
        return 1
    fi
    echo "Control-limit gate self-test: OK"
}

# ------------------------------------------------------------- dispatch

case "${1:-}" in
    --self-test)
        self_test
        ;;
    '')
        run_checks
        ;;
    *)
        echo "usage: $0 [--self-test]" >&2
        exit 2
        ;;
esac
