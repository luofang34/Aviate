#!/bin/bash
set -e

# Region+Branch Coverage Verification
# ====================================
# Measures LLVM source-based region (statement/line) and branch coverage of
# the kernel core after documented COV:EXCL exclusions, and fails if either
# drops below its floor in the [coverage_floors] section of cert/floors.toml.
# This is NOT MC/DC and claims no DO-178C structural-coverage credit; MC/DC is
# tracked non-blocking by the nightly workflow.
#
# Exclusion markers in source code:
#   // COV:EXCL(reason)           - exclude single line
#   // COV:EXCL_START(reason)     - start excluded block
#   // COV:EXCL_STOP              - end excluded block
#   // COV:EXCL_FUNC(reason)      - exclude entire function (on fn line)
#   // COV:EXCL_BR(reason)        - exclude branch on this line
#
# LLVM branch artifacts (folded branches) are auto-detected and documented.

OUTPUT_DIR="${COVERAGE_OUTPUT:-target/coverage}"
PROF_DIR="target/profiles"
COVERAGE_MODE="${COVERAGE_MODE:-branch}"
EXCLUDE_DOC="aviate-core/coverage.exclude"
FLOORS_FILE="${FLOORS_FILE:-cert/floors.toml}"

# Read a floor from the [coverage_floors] section of cert/floors.toml.
# These floors live in their own section (not [floors]) because cargo-evidence
# reserves the [floors] namespace for dimensions it measures itself; the
# region/branch ratchet is enforced here (measured >= floor) and by
# scripts/floors-lower-lint.sh (a committed floor cannot drop without a
# `Lower-Floor:` justification line).
read_coverage_floor() {
    awk -v key="$1" '
        /^\[coverage_floors\][[:space:]]*$/ { in_sec = 1; next }
        /^\[/ { in_sec = 0 }
        in_sec && $1 == key {
            for (i = 1; i <= NF; i++) if ($i == "=") { print $(i + 1); exit }
        }
    ' "$FLOORS_FILE"
}

REGION_FLOOR="$(read_coverage_floor coverage_region)"
BRANCH_FLOOR="$(read_coverage_floor coverage_branch)"
: "${REGION_FLOOR:=100}"
: "${BRANCH_FLOOR:=100}"

echo "========================================"
echo "Aviate Region+Branch Coverage Analysis"
echo "========================================"
echo "Mode: ${COVERAGE_MODE} | Floors: region>=${REGION_FLOOR}% branch>=${BRANCH_FLOOR}%"
echo "========================================"

NIGHTLY_LLVM="$(dirname "$(rustc +nightly --print target-libdir)")/bin"

# Prerequisites
command -v grcov &>/dev/null || { echo "Error: grcov not found"; exit 1; }

if [[ "$COVERAGE_MODE" == "branch" || "$COVERAGE_MODE" == "condition" ]]; then
    rustup run nightly rustc --version &>/dev/null || { echo "Error: nightly required"; exit 1; }
    TOOLCHAIN="+nightly"
    rustup run nightly rustup component list | grep -q "llvm-tools.*(installed)" || \
        rustup run nightly rustup component add llvm-tools-preview
else
    TOOLCHAIN=""
fi

# Environment
mkdir -p "$OUTPUT_DIR" "$PROF_DIR"
rm -f "$PROF_DIR"/*.profraw

case "$COVERAGE_MODE" in
    block)     export RUSTFLAGS="-C instrument-coverage" ;;
    branch)    export RUSTFLAGS="-C instrument-coverage -Zcoverage-options=branch" ;;
    condition) export RUSTFLAGS="-C instrument-coverage -Zcoverage-options=condition" ;;
esac
export LLVM_PROFILE_FILE="$PROF_DIR/cov-%p-%m.profraw"

# Run Tests
echo ""
echo "--- Running Tests ---"
cargo $TOOLCHAIN test --workspace

# Merge Profiles
echo ""
PROFRAW_COUNT=$(find . -name "*.profraw" 2>/dev/null | wc -l)
echo "Merging $PROFRAW_COUNT profile files..."
"$NIGHTLY_LLVM/llvm-profdata" merge -sparse $(find . -name "*.profraw") -o "$PROF_DIR/merged.profdata"

# Generate Reports
echo "Generating coverage reports..."
LLVM_PATH=""
[[ -d "$NIGHTLY_LLVM" ]] && LLVM_PATH="--llvm-path $NIGHTLY_LLVM"

grcov . \
    --binary-path ./target/debug/ \
    --source-dir . \
    --output-type lcov \
    --branch \
    --ignore-not-existing \
    --ignore "/*" \
    --ignore "tests/*" \
    --ignore "external/*" \
    --ignore "aviate-apps/*" \
    --ignore "aviate-hal/*" \
    --keep-only "aviate-core/src/*" \
    --keep-only "aviate-core/src/**/*" \
    --excl-line "COV:EXCL" \
    --excl-start "COV:EXCL_START" \
    --excl-stop "COV:EXCL_STOP" \
    --excl-br-line "COV:EXCL" \
    --excl-br-start "COV:EXCL_START" \
    --excl-br-stop "COV:EXCL_STOP" \
    $LLVM_PATH \
    --output-path "$OUTPUT_DIR/lcov.info"

# Also generate HTML
grcov . \
    --binary-path ./target/debug/ \
    --source-dir . \
    --output-type html \
    --branch \
    --ignore-not-existing \
    --ignore "/*" \
    --ignore "tests/*" \
    --ignore "external/*" \
    --ignore "aviate-apps/*" \
    --ignore "aviate-hal/*" \
    --keep-only "aviate-core/src/*" \
    --keep-only "aviate-core/src/**/*" \
    --excl-line "COV:EXCL" \
    --excl-start "COV:EXCL_START" \
    --excl-stop "COV:EXCL_STOP" \
    --excl-br-line "COV:EXCL" \
    --excl-br-start "COV:EXCL_START" \
    --excl-br-stop "COV:EXCL_STOP" \
    $LLVM_PATH \
    --output-path "$OUTPUT_DIR/html"

# Strip phantom DA/BRDA entries that point past the end of a source file.
# Rust re-exports (pub use foo::Bar) cause rustc's coverage debug info to
# attribute inlined items to BOTH the re-exporting file and the defining
# file — but with the DEFINING file's line numbers. In lcov.info this
# shows up as DA entries under SF:control.rs at lines that only exist in
# control/enums.rs, which later reports as "uncovered" in a shorter file.
# Filter them out once before any downstream aggregation sees them.
awk '
/^SF:/ {
    f = $0
    sub(/^SF:/, "", f)
    fl = 0
    if ((getline line < f) >= 0) {
        close(f)
        cmd = "wc -l < \"" f "\" 2>/dev/null"
        cmd | getline fl
        close(cmd)
        fl = fl + 0
    }
    print
    next
}
/^DA:/ || /^BRDA:/ {
    if (fl > 0) {
        split($0, a, ":")
        split(a[2], b, ",")
        ln = b[1] + 0
        if (ln > fl) next
    }
    print
    next
}
{ print }
' "$OUTPUT_DIR/lcov.info" > "$OUTPUT_DIR/lcov.info.filtered"
mv "$OUTPUT_DIR/lcov.info.filtered" "$OUTPUT_DIR/lcov.info"

# Calculate metrics
read -r LINE_HIT LINE_TOTAL BRANCH_HIT BRANCH_TOTAL < <(
    awk '
    BEGIN { lh=0; lt=0; bh=0; bt=0; f="" }
    /^SF:/ { f=$0; sub(/^SF:/, "", f) }
    /^DA:/ && f ~ /\/src\// { sub(/^DA:/, ""); split($0,a,","); lt++; if(a[2]>0) lh++ }
    /^BRDA:/ && f ~ /\/src\// { sub(/^BRDA:/, ""); split($0,a,","); bt++; if(a[4]!="-" && a[4]!="0") bh++ }
    END { print lh, lt, bh, bt }
    ' "$OUTPUT_DIR/lcov.info"
)

# Function coverage (deduplicated) - use MAX hit count per function
# LLVM generates multiple entries for generic functions (with placeholder and concrete types)
# We take the maximum hit count across all instantiations to avoid false uncovered reports
read -r FUNC_HIT FUNC_TOTAL < <(
    awk '
    BEGIN { f="" }
    /^SF:/ { f=$0; sub(/^SF:/, "", f) }
    /^FNDA:/ && f ~ /\/src\// {
        sub(/^FNDA:/, "")
        # Split only on first comma (function names may contain commas in generics)
        idx = index($0, ",")
        count = substr($0, 1, idx-1)
        name = substr($0, idx+1)
        gsub(/<[^>]+>/, "<T>", name)
        key = f ":" name
        if (!(key in seen)) { seen[key]=1; hits[key]=0 }
        if (count > hits[key]) { hits[key] = count }
    }
    END {
        total = 0; hit = 0
        for (k in seen) { total++; if (hits[k] > 0) hit++ }
        print hit, total
    }
    ' "$OUTPUT_DIR/lcov.info"
)

# Count exclusion markers
LINE_EXCL=$(grep -r "COV:EXCL" aviate-core/src --include="*.rs" 2>/dev/null | grep -v "_STOP" | wc -l || echo 0)

# Count excluded functions by looking for COV:EXCL blocks that contain "pub fn" or "fn "
# This accounts for LLVM tracking function coverage separately from line coverage
FUNC_EXCL=0
for f in aviate-core/src/*.rs aviate-core/src/**/*.rs; do
    [[ -f "$f" ]] || continue
    # Count functions inside exclusion blocks using awk
    count=$(awk '
        /COV:EXCL_START/ { in_block = 1 }
        /COV:EXCL_STOP/ { in_block = 0 }
        in_block && /pub fn |^[[:space:]]*fn / { funcs++ }
        /COV:EXCL\(/ && /fn / { funcs++ }
        END { print funcs+0 }
    ' "$f" 2>/dev/null)
    FUNC_EXCL=$((FUNC_EXCL + count))
done

# Count COV:EXCL_START blocks (for reporting)
EXCL_BLOCKS=$(grep -r "COV:EXCL_START" aviate-core/src --include="*.rs" 2>/dev/null | wc -l || echo 0)

# Detect LLVM branch artifacts
BRANCH_ARTIFACTS=0
if [[ "$COVERAGE_MODE" != "block" && $BRANCH_HIT -lt $BRANCH_TOTAL ]]; then
    BINARY=$(find target/debug/deps -maxdepth 1 -name "aviate_core-*" -type f -executable 2>/dev/null | head -1)
    if [[ -n "$BINARY" && -f "$PROF_DIR/merged.profdata" ]]; then
        LLVM_OUT=$("$NIGHTLY_LLVM/llvm-cov" show "$BINARY" \
            --instr-profile="$PROF_DIR/merged.profdata" \
            --show-branches=count \
            aviate-core/src/lib.rs aviate-core/src/ekf.rs aviate-core/src/mixer.rs 2>&1 || true)
        BRANCH_ARTIFACTS=$(echo "$LLVM_OUT" | grep -c "Folded - Ignored" 2>/dev/null) || BRANCH_ARTIFACTS=0
    fi
fi

# Calculate final percentages
LINE_UNCOV=$((LINE_TOTAL - LINE_HIT))
FUNC_UNCOV=$((FUNC_TOTAL - FUNC_HIT))
BRANCH_UNCOV=$((BRANCH_TOTAL - BRANCH_HIT))

# After exclusions
LINE_FINAL=$(awk "BEGIN {if($LINE_TOTAL>0) printf \"%.1f\", $LINE_HIT/$LINE_TOTAL*100; else print 100}")

# Function coverage: adjust for excluded functions
# FUNC_EXCL counts functions in COV:EXCL blocks in source code
# If hit >= effective_total, all uncovered functions are excluded
FUNC_EFFECTIVE_TOTAL=$((FUNC_TOTAL - FUNC_EXCL))
if [[ $FUNC_EFFECTIVE_TOTAL -lt 0 ]]; then FUNC_EFFECTIVE_TOTAL=0; fi
# Cap at 100% when hit >= effective_total (means all uncovered are in exclusions)
if [[ $FUNC_HIT -ge $FUNC_EFFECTIVE_TOTAL ]]; then
    FUNC_FINAL="100.0"
else
    FUNC_FINAL=$(awk "BEGIN {if($FUNC_EFFECTIVE_TOTAL>0) printf \"%.1f\", $FUNC_HIT/$FUNC_EFFECTIVE_TOTAL*100; else print 100}")
fi

# Branches: account for LLVM artifacts
BRANCH_EFFECTIVE_UNCOV=$((BRANCH_UNCOV - BRANCH_ARTIFACTS))
if [[ $BRANCH_EFFECTIVE_UNCOV -lt 0 ]]; then BRANCH_EFFECTIVE_UNCOV=0; fi
# Cap at 100.0% when artifacts account for all uncovered branches —
# matches the treatment of Functions when FUNC_HIT >= FUNC_EFFECTIVE_TOTAL.
# Without this cap, (hit + artifacts) / total can drift above 1.0 and the
# string compare `!= "100.0"` below flags a genuinely-complete run as a
# failure.
if [[ $BRANCH_EFFECTIVE_UNCOV -eq 0 ]]; then
    BRANCH_FINAL="100.0"
else
    BRANCH_FINAL=$(awk "BEGIN {if($BRANCH_TOTAL>0) printf \"%.1f\", ($BRANCH_HIT+$BRANCH_ARTIFACTS)/$BRANCH_TOTAL*100; else print 100}")
fi

# Summary
echo ""
echo "========================================"
echo "Coverage Summary"
echo "========================================"
echo ""
echo "  Lines:     $LINE_HIT/$LINE_TOTAL"
if [[ $LINE_UNCOV -gt 0 ]]; then
    echo "             + $LINE_EXCL excluded via COV:EXCL markers"
fi
echo "             = $LINE_FINAL%"
echo ""
echo "  Functions: $FUNC_HIT/$FUNC_TOTAL [deduplicated]"
if [[ $FUNC_EXCL -gt 0 ]]; then
    echo "             - $FUNC_EXCL excluded (in COV:EXCL blocks)"
    echo "             = $FUNC_HIT/$FUNC_EFFECTIVE_TOTAL = $FUNC_FINAL%"
else
    echo "             = $FUNC_FINAL%"
fi
echo ""
echo "  Branches:  $BRANCH_HIT/$BRANCH_TOTAL"
if [[ $BRANCH_ARTIFACTS -gt 0 ]]; then
    echo "             + $BRANCH_ARTIFACTS LLVM artifacts (folded branches)"
fi
echo "             = $BRANCH_FINAL%"
echo ""
echo "  Documentation: $EXCLUDE_DOC"
echo "========================================"

# Generate exclusion documentation
cat > "$EXCLUDE_DOC" << 'HEADER'
# Coverage Exclusions for aviate-core
# ====================================
# Auto-generated. All exclusions use inline markers that move with code.
#
# Marker types:
#   COV:EXCL(reason)       - exclude line
#   COV:EXCL_START(reason) - start block
#   COV:EXCL_STOP          - end block
#
# Categories:
#   STUB      - Placeholder, not implemented
#   DEFAULT   - Default impl for optional feature
#   DEFENSIVE - Safety guard, cannot trigger in unit tests
#   EMPTY     - Empty block (no executable code)

HEADER

echo "## Summary" >> "$EXCLUDE_DOC"
echo "" >> "$EXCLUDE_DOC"
echo "| Metric | Covered | Total | Excluded | Final |" >> "$EXCLUDE_DOC"
echo "|--------|---------|-------|----------|-------|" >> "$EXCLUDE_DOC"
echo "| Lines | $LINE_HIT | $LINE_TOTAL | $LINE_EXCL markers | $LINE_FINAL% |" >> "$EXCLUDE_DOC"
echo "| Functions | $FUNC_HIT | $FUNC_TOTAL | $FUNC_EXCL (STUB/DEFAULT) | $FUNC_FINAL% |" >> "$EXCLUDE_DOC"
echo "| Branches | $BRANCH_HIT | $BRANCH_TOTAL | $BRANCH_ARTIFACTS LLVM artifacts | $BRANCH_FINAL% |" >> "$EXCLUDE_DOC"
echo "" >> "$EXCLUDE_DOC"

echo "## Exclusions by Category" >> "$EXCLUDE_DOC"
echo "" >> "$EXCLUDE_DOC"
for cat in STUB DEFAULT DEFENSIVE EMPTY; do
    COUNT=$(grep -r "COV:EXCL.*$cat" aviate-core/src --include="*.rs" 2>/dev/null | wc -l || echo 0)
    echo "- **$cat**: $COUNT" >> "$EXCLUDE_DOC"
done
echo "" >> "$EXCLUDE_DOC"

if [[ $BRANCH_ARTIFACTS -gt 0 ]]; then
    echo "## LLVM Branch Artifacts" >> "$EXCLUDE_DOC"
    echo "" >> "$EXCLUDE_DOC"
    echo "$BRANCH_ARTIFACTS branches marked as 'Folded - Ignored' by LLVM." >> "$EXCLUDE_DOC"
    echo "These are instrumentation artifacts, not real code decisions." >> "$EXCLUDE_DOC"
    echo "Verified via: llvm-cov show --show-branches=count" >> "$EXCLUDE_DOC"
    echo "" >> "$EXCLUDE_DOC"
fi

echo "## Detailed Exclusion List" >> "$EXCLUDE_DOC"
echo "" >> "$EXCLUDE_DOC"
for f in aviate-core/src/*.rs aviate-core/src/**/*.rs; do
    [[ -f "$f" ]] || continue
    MARKERS=$(grep -n "COV:EXCL" "$f" 2>/dev/null || true)
    if [[ -n "$MARKERS" ]]; then
        echo "### ${f#aviate-core/src/}" >> "$EXCLUDE_DOC"
        echo '```' >> "$EXCLUDE_DOC"
        echo "$MARKERS" >> "$EXCLUDE_DOC"
        echo '```' >> "$EXCLUDE_DOC"
        echo "" >> "$EXCLUDE_DOC"
    fi
done

# Check pass/fail
echo ""
FAILED=0

# Region coverage is LLVM source-based statement/line coverage (grcov DA
# entries) of aviate-core/src after COV:EXCL exclusions. It must not drop
# below the coverage_region floor.
REGION_BELOW=$(awk -v m="$LINE_FINAL" -v f="$REGION_FLOOR" 'BEGIN { print (m + 0 < f + 0) ? 1 : 0 }')
if [[ "$REGION_BELOW" == "1" ]]; then
    echo "FAILED: Region $LINE_FINAL% < floor ${REGION_FLOOR}%"
    echo "  Lower coverage_region in cert/floors.toml with a 'Lower-Floor:' line, or add tests/exclusions."
    echo "  Uncovered lines not excluded:"
    awk '
    /^SF:/ { f=$0; sub(/^SF:.*\//, "", f) }
    /^DA:/ && f != "" {
        sub(/^DA:/, ""); split($0,a,",")
        if(a[2]==0) print "    " f ":" a[1]
    }' "$OUTPUT_DIR/lcov.info" | head -10
    FAILED=1
fi

if [[ "$FUNC_FINAL" != "100.0" ]]; then
    echo "FAILED: Functions $FUNC_FINAL% < 100%"
    echo "  Uncovered functions (deduplicated):"
    # Use same MAX hit logic as counting - only report truly uncovered functions
    awk '
    BEGIN { f="" }
    /^SF:/ { f=$0; sub(/^SF:/, "", f) }
    /^FNDA:/ && f ~ /\/src\// {
        sub(/^FNDA:/, "")
        # Split only on first comma (function names may contain commas in generics)
        idx = index($0, ",")
        count = substr($0, 1, idx-1)
        name = substr($0, idx+1)
        gsub(/<[^>]+>/,"<T>",name)
        key=f":"name
        if (!(key in seen)) { seen[key]=1; hits[key]=0; fname[key]=f; funcname[key]=name }
        if (count > hits[key]) { hits[key] = count }
    }
    END {
        for (k in seen) {
            if (hits[k] == 0) { print "    " fname[k] ": " substr(funcname[k],1,60) }
        }
    }' "$OUTPUT_DIR/lcov.info" | head -10
    FAILED=1
fi

BRANCH_BELOW=$(awk -v m="$BRANCH_FINAL" -v f="$BRANCH_FLOOR" 'BEGIN { print (m + 0 < f + 0) ? 1 : 0 }')
if [[ "$BRANCH_BELOW" == "1" ]]; then
    echo "FAILED: Branches $BRANCH_FINAL% < floor ${BRANCH_FLOOR}%"
    echo "  $BRANCH_EFFECTIVE_UNCOV real uncovered branches (after excluding $BRANCH_ARTIFACTS LLVM artifacts)"
    echo "  Lower coverage_branch in cert/floors.toml with a 'Lower-Floor:' line, or add tests/exclusions."
    FAILED=1
fi

if [[ $FAILED -eq 1 ]]; then
    echo ""
    echo "Add COV:EXCL markers to exclude with justification, or add tests."
    exit 1
fi

echo "========================================"
echo "PASSED: region ${LINE_FINAL}% >= ${REGION_FLOOR}%, branch ${BRANCH_FINAL}% >= ${BRANCH_FLOOR}%"
echo "========================================"
exit 0
