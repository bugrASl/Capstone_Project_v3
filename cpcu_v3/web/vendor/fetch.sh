#!/usr/bin/env bash
##
##  fetch.sh — pull pinned Mongoose source into this directory.
##  Re-run any time you want to upgrade the version (edit MONGOOSE_VER below).
##  Author: bugrASl
##

set -euo pipefail

MONGOOSE_VER="7.14"
BASE="https://raw.githubusercontent.com/cesanta/mongoose/${MONGOOSE_VER}"

cd "$(dirname "$0")"

echo "Fetching Mongoose ${MONGOOSE_VER}..."
curl -fsSL -o mongoose.h "${BASE}/mongoose.h"
curl -fsSL -o mongoose.c "${BASE}/mongoose.c"
echo "Done. Files:"
wc -l mongoose.c mongoose.h
echo ""
echo "Re-run 'cmake -S . -B build' from the project root to enable the cpcu_ws target."
