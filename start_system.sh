#!/bin/bash

WORKING_DIGEST="sha256:6429aec3c008a8912b52192f8f88de770518a29af2916b0648286c76b4711494"
CONTAINER_ID=$(docker ps -q --filter "ancestor=ghcr.io/nautilus-unipd/raspberry-setup@${WORKING_DIGEST}" | head -n 1)

if [ -n "$CONTAINER_ID" ]; then
    docker exec -it "$CONTAINER_ID" bash
else
    # Pull the specific version by digest
    docker pull ghcr.io/nautilus-unipd/raspberry-setup@${WORKING_DIGEST}
    docker compose run --rm raspberry
fi
