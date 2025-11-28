#!/bin/bash
# Coverage report generator for aviate-core
# Generates detailed coverage reports for DO-178C documentation

set -e

OUTPUT_DIR="${1:-target/coverage}"
mkdir -p "$OUTPUT_DIR"

echo "========================================"
echo "Aviate Coverage Report Generator"
echo "========================================"
echo ""

# Run tarpaulin with multiple output formats
echo "Running coverage analysis..."
cargo tarpaulin \
    --packages aviate-core \
    --out Html \
    --out Json \
    --out Xml \
    --output-dir "$OUTPUT_DIR" \
    --skip-clean \
    2>&1

echo ""
echo "========================================"
echo "Coverage Summary"
echo "========================================"

# Parse and display per-module coverage
cargo tarpaulin --packages aviate-core --out Stdout --skip-clean 2>&1 | grep -E "^|| aviate-core" | while read line; do
    echo "$line"
done

echo ""
echo "========================================"
echo "Reports Generated"
echo "========================================"
echo "HTML Report: $OUTPUT_DIR/tarpaulin-report.html"
echo "JSON Report: $OUTPUT_DIR/tarpaulin-report.json"
echo "XML Report:  $OUTPUT_DIR/cobertura.xml"
echo ""
echo "Open HTML report with: xdg-open $OUTPUT_DIR/tarpaulin-report.html"
