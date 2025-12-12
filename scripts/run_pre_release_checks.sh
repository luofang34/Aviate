#!/bin/bash
set -e

# Master script for pre-release checks
# DO-178C DAL-A/B compliance verification

echo "========================================"
echo "Starting Pre-Release Checks"
echo "========================================"

# 1. Run Tests
echo ">> Running Test Suite..."
cargo test --workspace

# 2. Check Coverage (DO-178C requirement)
echo ">> Running Coverage Analysis..."
COVERAGE_THRESHOLD=${COVERAGE_THRESHOLD:-100}
cargo tarpaulin --packages aviate-core --fail-under $COVERAGE_THRESHOLD --out Stdout

# 3. Check Memory Limits (Flash/RAM footprint)
echo ">> Running Memory Limit Checks..."
./scripts/check_memory_limits.sh

# 4. Run SITL Flight Test
echo ">> Running SITL Flight Test..."
./scripts/run_sitl.sh

# Future checks can be added here (e.g., static analysis, special tests)
# echo ">> Running Static Analysis..."
# ./scripts/static_analysis.sh

echo "========================================"
echo "All Pre-Release Checks Passed"
echo "========================================"
