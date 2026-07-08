#!/bin/bash
# Convenience launcher: allows the container to draw on the host X server,
# then starts (or attaches to) the argos3 dev container.
set -e

xhost +local:docker >/dev/null

docker compose up -d
docker compose exec argos3 /usr/local/bin/entrypoint.sh bash

# Revoke X access again once you're done (optional, tightens security):
# xhost -local:docker
