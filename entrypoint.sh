#!/bin/bash
set -e

SRC=/opt/argos3-src
DEST="$HOME/workspace/argos3"

# First run: the bind-mounted ./workspace (host) is empty, so seed it with
# the baked ARGoS3 core source under workspace/argos3/. After this, the
# source lives on the host and can be edited with VS Code; edits persist
# across containers. Other projects (argos3-examples, your own code) live
# alongside it under ./workspace/ and are never touched by this script.
mkdir -p "$DEST"
if [ -z "$(ls -A "$DEST" 2>/dev/null)" ]; then
    echo "[entrypoint] Populating $DEST with ARGoS3 source (first run)..."
    cp -r "$SRC"/. "$DEST"/
    echo "[entrypoint] Done. Build it with:"
    echo "    cd ~/workspace/argos3 && mkdir -p build_simulator && cd build_simulator"
    echo "    cmake ../src -DCMAKE_BUILD_TYPE=Release && make -j\$(nproc)"
fi

exec "$@"
