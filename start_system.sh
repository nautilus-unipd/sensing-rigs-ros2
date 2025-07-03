#!/bin/bash

CONTAINER_ID=$(docker ps -q --filter "ancestor=ghcr.io/nautilus-unipd/raspberry-setup:latest" | head -n 1)

if [ -n "$CONTAINER_ID" ]; then
    docker exec -it "$CONTAINER_ID" bash
else
    docker compose run --rm raspberry
fi