#!/usr/bin/env bash
# Builds hover_core with the host compiler and runs the tests (Ubuntu or WSL).
set -euo pipefail
cd "$(dirname "$0")/.."
g++ -std=c++17 -Wall -Wextra -I components/hover_core/include \
    components/hover_core/src/*.cpp test_host/test_core.cpp -o /tmp/hover_core_tests
/tmp/hover_core_tests
