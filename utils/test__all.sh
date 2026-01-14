#!/bin/bash
# Passform2/utils/test__all.sh
set -e # Exit on error

TEST_DIR="chubidubidu"

echo "Running all unittests in $TEST_DIR using unittest discovery..."

python3 -m unittest discover -s "$TEST_DIR" -p 'test__*.py'

echo "All unittests completed successfully."
