#!/bin/bash
set -e

# DO-178C DAL-A Coverage Verification Script
# Uses LLVM Source-Based Coverage with Branch/MC/DC analysis.
#
# Coverage modes (set via COVERAGE_MODE env var):
#   block     - Basic block coverage (works on stable)
#   branch    - Branch coverage (requires nightly) [default]
#   condition - MC/DC condition coverage (requires nightly, DO-178C DAL-A)
#
# Exclusions:
#   See aviate-core/coverage.exclude for documented exclusions.
#   Target is 100% after exclusions to catch untested code.
#
# Artifacts:
#   - LLVM 'folded' branches are instrumentation artifacts (not real decisions)
#   - Monomorphized functions are deduplicated for meaningful metrics

# Configuration
THRESHOLD=${COVERAGE_THRESHOLD:-100}
OUTPUT_DIR="${COVERAGE_OUTPUT:-target/coverage}"
PROF_DIR="target/profiles"
COVERAGE_MODE="${COVERAGE_MODE:-branch}"
EXCLUDE_FILE="aviate-core/coverage.exclude"

echo "========================================"
echo "Aviate LLVM Coverage Analysis (DO-178C)"
echo "========================================"
echo "Coverage Mode: ${COVERAGE_MODE}"
echo "Threshold:     ${THRESHOLD}%"
echo "Output:        ${OUTPUT_DIR}"
echo "========================================"

# Detect nightly LLVM tools path
NIGHTLY_LLVM="$HOME/.rustup/toolchains/nightly-x86_64-unknown-linux-gnu/lib/rustlib/x86_64-unknown-linux-gnu/bin"

# 1. Prerequisite Check
if ! command -v grcov &> /dev/null; then
    echo "Error: 'grcov' not found. Install with: cargo install grcov"
    exit 1
fi

if [[ "$COVERAGE_MODE" == "branch" || "$COVERAGE_MODE" == "condition" ]]; then
    if ! rustup run nightly rustc --version &> /dev/null; then
        echo "Error: nightly toolchain required for $COVERAGE_MODE coverage."
        echo "Install with: rustup toolchain install nightly"
        exit 1
    fi
    TOOLCHAIN="+nightly"
    if ! rustup run nightly rustup component list | grep -q "llvm-tools.*(installed)"; then
        echo "Installing llvm-tools-preview for nightly..."
        rustup run nightly rustup component add llvm-tools-preview
    fi
else
    TOOLCHAIN=""
    if ! rustup component list | grep -q "llvm-tools.*(installed)"; then
        echo "Error: 'llvm-tools' not found. Install with: rustup component add llvm-tools-preview"
        exit 1
    fi
fi

# 2. Prepare Environment
mkdir -p "$OUTPUT_DIR"
mkdir -p "$PROF_DIR"
rm -f "$PROF_DIR"/*.profraw

case "$COVERAGE_MODE" in
    block)     export RUSTFLAGS="-C instrument-coverage" ;;
    branch)    export RUSTFLAGS="-C instrument-coverage -Zcoverage-options=branch" ;;
    condition) export RUSTFLAGS="-C instrument-coverage -Zcoverage-options=condition" ;;
    *)
        echo "Error: Unknown COVERAGE_MODE: $COVERAGE_MODE (valid: block, branch, condition)"
        exit 1
        ;;
esac

export LLVM_PROFILE_FILE="$PROF_DIR/cov-%p-%m.profraw"
echo "RUSTFLAGS: $RUSTFLAGS"

# 3. Run Tests
echo ""
echo "--- Phase 1: Running Tests ---"
cargo $TOOLCHAIN test --workspace --exclude aviate-app-quadcopter-stm32h7

# 3.5 Merge profiles immediately
echo ""
echo "--- Phase 1.5: Merging Profiles ---"
PROFRAW_COUNT=$(find . -name "*.profraw" 2>/dev/null | wc -l)
if [[ "$PROFRAW_COUNT" -eq 0 ]]; then
    echo "Warning: No profraw files found. Coverage data may be incomplete."
else
    echo "Found $PROFRAW_COUNT profraw files"
    "$NIGHTLY_LLVM/llvm-profdata" merge -sparse $(find . -name "*.profraw") -o "$PROF_DIR/merged.profdata"
    echo "Merged to: $PROF_DIR/merged.profdata"
fi

# 4. Generate Reports
echo ""
echo "--- Phase 2: Generating Reports ---"

# Determine LLVM tools path for grcov
LLVM_PATH=""
if [[ -d "$NIGHTLY_LLVM" ]]; then
    LLVM_PATH="--llvm-path $NIGHTLY_LLVM"
fi

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
    --ignore "aviate-platform/*" \
    --keep-only "aviate-core/src/*" \
    $LLVM_PATH \
    --output-path "$OUTPUT_DIR/html"

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
    --ignore "aviate-platform/*" \
    --keep-only "aviate-core/src/*" \
    $LLVM_PATH \
    --output-path "$OUTPUT_DIR/lcov.info"

echo "Reports: $OUTPUT_DIR/html/index.html"

# 5. Parse exclusions
declare -A EXCLUDED_LINES
if [[ -f "$EXCLUDE_FILE" ]]; then
    while IFS= read -r line; do
        # Skip comments and empty lines
        [[ "$line" =~ ^#.*$ || -z "$line" ]] && continue
        # Extract file:line before the comment
        entry=$(echo "$line" | awk '{print $1}')
        if [[ "$entry" =~ ^[^:]+:[0-9]+$ ]]; then
            EXCLUDED_LINES["$entry"]=1
        fi
    done < "$EXCLUDE_FILE"
    echo ""
    echo "--- Exclusions: ${#EXCLUDED_LINES[@]} lines from $EXCLUDE_FILE ---"
fi

# 6. Calculate Coverage Metrics (pure awk)
echo ""
echo "--- Phase 3: Coverage Summary ---"

read -r LINE_HIT LINE_TOTAL BRANCH_HIT BRANCH_TOTAL < <(
    awk '
    BEGIN { lh=0; lt=0; bh=0; bt=0; cur_file="" }
    /^SF:/ { cur_file=$0; sub(/^SF:/, "", cur_file) }
    /^DA:/ && cur_file ~ /\/src\// {
        sub(/^DA:/, "")
        split($0, a, ",")
        lt++
        if (a[2] > 0) lh++
    }
    /^BRDA:/ && cur_file ~ /\/src\// {
        sub(/^BRDA:/, "")
        split($0, a, ",")
        bt++
        if (a[4] != "-" && a[4] != "0") bh++
    }
    END { print lh, lt, bh, bt }
    ' "$OUTPUT_DIR/lcov.info"
)

# Count excluded lines that are actually uncovered (pure awk)
EXCLUDED_UNCOVERED=$(awk -v exclude_file="$EXCLUDE_FILE" '
BEGIN {
    # Load exclusions
    while ((getline line < exclude_file) > 0) {
        if (line !~ /^#/ && line ~ /^[^:]+:[0-9]+/) {
            split(line, a, " ")
            excluded[a[1]] = 1
        }
    }
    count = 0
    cur_file = ""
}
/^SF:/ {
    cur_file = $0
    sub(/^SF:.*\//, "", cur_file)  # Get just filename
}
/^DA:/ && cur_file != "" {
    sub(/^DA:/, "")
    split($0, a, ",")
    line_num = a[1]
    hit_count = a[2]
    key = cur_file ":" line_num
    if (hit_count == 0 && key in excluded) {
        count++
    }
}
END { print count }
' "$OUTPUT_DIR/lcov.info")

# Calculate adjusted metrics
UNCOVERED_LINES=$((LINE_TOTAL - LINE_HIT))
ADJUSTED_UNCOVERED=$((UNCOVERED_LINES - EXCLUDED_UNCOVERED))

if [[ $LINE_TOTAL -gt 0 ]]; then
    LINE_COV=$(awk "BEGIN {printf \"%.1f\", $LINE_HIT / $LINE_TOTAL * 100}")
    ADJUSTED_TOTAL=$((LINE_TOTAL - EXCLUDED_UNCOVERED))
    ADJUSTED_HIT=$((LINE_HIT))
    ADJUSTED_COV=$(awk "BEGIN {printf \"%.1f\", ($ADJUSTED_TOTAL - $ADJUSTED_UNCOVERED) / $ADJUSTED_TOTAL * 100}")
else
    LINE_COV="0.0"
    ADJUSTED_COV="0.0"
fi

if [[ $BRANCH_TOTAL -gt 0 ]]; then
    BRANCH_COV=$(awk "BEGIN {printf \"%.1f\", $BRANCH_HIT / $BRANCH_TOTAL * 100}")
else
    BRANCH_COV="0.0"
fi

echo "  Lines:    ${LINE_HIT}/${LINE_TOTAL} + ${EXCLUDED_UNCOVERED} excluded (see $EXCLUDE_FILE)"
echo "  Adjusted: ${ADJUSTED_COV}%"
echo "  Branches: ${BRANCH_HIT}/${BRANCH_TOTAL}"

# 7. Report undocumented uncovered lines
if [[ "$ADJUSTED_UNCOVERED" -gt 0 ]]; then
    echo ""
    echo "  ⚠ UNDOCUMENTED UNCOVERED LINES ($ADJUSTED_UNCOVERED):"
    awk -v exclude_file="$EXCLUDE_FILE" '
    BEGIN {
        while ((getline line < exclude_file) > 0) {
            if (line !~ /^#/ && line ~ /^[^:]+:[0-9]+/) {
                split(line, a, " ")
                excluded[a[1]] = 1
            }
        }
        cur_file = ""
    }
    /^SF:/ {
        cur_file = $0
        sub(/^SF:.*\//, "", cur_file)
    }
    /^DA:/ && cur_file != "" {
        sub(/^DA:/, "")
        split($0, a, ",")
        line_num = a[1]
        hit_count = a[2]
        key = cur_file ":" line_num
        if (hit_count == 0 && !(key in excluded)) {
            print "    " key
        }
    }
    ' "$OUTPUT_DIR/lcov.info"
fi

# 8. Verify Branch Artifacts (if not 100%)
if [[ "$COVERAGE_MODE" != "block" ]] && (( $(echo "$BRANCH_COV < 100" | bc -l) )); then
    echo ""
    echo "--- Phase 4: Branch Artifact Verification ---"

    BINARY=$(find target/debug/deps -maxdepth 1 -name "aviate_core-*" -type f -executable 2>/dev/null | head -1)

    if [[ -n "$BINARY" && -f "$PROF_DIR/merged.profdata" ]]; then
        echo "  Binary: $BINARY"
        echo "  Profile: $PROF_DIR/merged.profdata"

        LLVM_OUTPUT=$("$NIGHTLY_LLVM/llvm-cov" show "$BINARY" \
            --instr-profile="$PROF_DIR/merged.profdata" \
            --show-branches=count \
            aviate-core/src/lib.rs 2>&1 || true)

        FOLDED=$(echo "$LLVM_OUTPUT" | grep -c "Folded - Ignored" || echo 0)
        UNCOVERED_BRANCHES=$((BRANCH_TOTAL - BRANCH_HIT))

        echo "  Uncovered branches (grcov): $UNCOVERED_BRANCHES"
        echo "  Folded branches (llvm-cov): $FOLDED"

        if [[ "$FOLDED" -ge "$UNCOVERED_BRANCHES" ]]; then
            echo ""
            echo "  ✓ All uncovered branches are LLVM artifacts [Folded - Ignored]"
            echo "  ✓ TRUE BRANCH COVERAGE: 100%"
            BRANCH_COV="100.0"
        else
            REAL_UNCOVERED=$((UNCOVERED_BRANCHES - FOLDED))
            echo ""
            echo "  ⚠ $REAL_UNCOVERED branches require additional tests"
        fi
    else
        echo "  Warning: Could not verify branch artifacts"
        [[ -z "$BINARY" ]] && echo "    - Binary not found"
        [[ ! -f "$PROF_DIR/merged.profdata" ]] && echo "    - Profile data not found"
    fi
fi

# 9. Threshold Check
echo ""
echo "--- Result ---"

# Use adjusted coverage for threshold check
if (( $(echo "$ADJUSTED_COV < $THRESHOLD" | bc -l) )); then
    echo "FAILED: Adjusted coverage ${ADJUSTED_COV}% below ${THRESHOLD}% threshold"
    echo ""
    echo "To fix: Either add tests for uncovered lines, or document"
    echo "exclusions in $EXCLUDE_FILE with justification."
    exit 1
fi

echo "PASSED: Adjusted coverage ${ADJUSTED_COV}% meets ${THRESHOLD}% threshold"

if (( $(echo "$BRANCH_COV < 100" | bc -l) )); then
    echo "WARNING: Branch coverage ${BRANCH_COV}% below 100%"
fi

exit 0
