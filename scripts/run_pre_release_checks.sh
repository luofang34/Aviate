#!/bin/bash
set -e

# Master script for pre-release checks

echo "========================================"
echo "Starting Pre-Release Checks"
echo "========================================"

# 1. Check Memory Limits (Flash/RAM footprint)
echo ">> Running Memory Limit Checks..."
./scripts/check_memory_limits.sh

# Future checks can be added here (e.g., static analysis, special tests)
# echo ">> Running Static Analysis..."
# ./scripts/static_analysis.sh

echo "========================================"
echo "All Pre-Release Checks Passed ✅"
echo "========================================"
