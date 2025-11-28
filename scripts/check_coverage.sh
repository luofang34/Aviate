#!/bin/bash
set -e

# Coverage check script for DO-178C compliance
# Enforces minimum line coverage threshold

THRESHOLD=${COVERAGE_THRESHOLD:-80}
OUTPUT_DIR="${COVERAGE_OUTPUT:-target/coverage}"

echo "========================================"
echo "Running Coverage Analysis"
echo "Threshold: ${THRESHOLD}%"
echo "========================================"

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Run tarpaulin and capture output
# --skip-clean avoids unnecessary rebuilds
# --out Json for parsing, Stdout for display
COVERAGE_OUTPUT=$(cargo tarpaulin --skip-clean \
    --out Json \
    --output-dir "$OUTPUT_DIR" \
    --packages aviate-core \
    2>&1)

# Extract coverage percentage from JSON
COVERAGE_JSON="$OUTPUT_DIR/tarpaulin-report.json"
if [ ! -f "$COVERAGE_JSON" ]; then
    echo "ERROR: Coverage report not generated"
    exit 1
fi

# Parse coverage percentage (line coverage)
COVERAGE=$(python3 -c "
import json
with open('$COVERAGE_JSON') as f:
    data = json.load(f)
    covered = sum(1 for f in data['files'] for l in f.get('traces', []) if l.get('hits', 0) > 0)
    total = sum(len(f.get('traces', [])) for f in data['files'])
    if total > 0:
        print(f'{(covered / total) * 100:.2f}')
    else:
        print('0.00')
" 2>/dev/null || echo "0.00")

echo ""
echo "========================================"
echo "Coverage Results"
echo "========================================"
echo "Line Coverage: ${COVERAGE}%"
echo "Threshold:     ${THRESHOLD}%"
echo ""

# Check threshold
if [ "$(echo "$COVERAGE < $THRESHOLD" | bc -l)" -eq 1 ]; then
    echo "FAILED: Coverage ${COVERAGE}% is below threshold ${THRESHOLD}%"
    echo ""
    echo "Uncovered files summary:"
    cargo tarpaulin --skip-clean --out Stdout --packages aviate-core 2>&1 | grep "Uncovered Lines:" -A 100 | head -30
    exit 1
fi

echo "PASSED: Coverage ${COVERAGE}% meets threshold ${THRESHOLD}%"
echo "========================================"

# Generate HTML report if requested
if [ "$GENERATE_HTML" = "true" ]; then
    echo "Generating HTML report..."
    cargo tarpaulin --skip-clean --out Html --output-dir "$OUTPUT_DIR" --packages aviate-core 2>&1
    echo "HTML report: $OUTPUT_DIR/tarpaulin-report.html"
fi

exit 0
