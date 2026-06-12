#!/usr/bin/env bash
set -euo pipefail

# Build only if image doesn't exist (first-time setup on a new Jetson).
# Skip if already built (no internet needed on subsequent runs).
if ! docker image inspect ece191/camera:humble >/dev/null 2>&1; then
    echo "[INFO] Image not found. Building for the first time..."
    if docker compose version >/dev/null 2>&1; then
        docker compose -f docker-compose.yml -f docker-compose.arm64.yml build camera
    elif command -v docker-compose >/dev/null 2>&1; then
        docker-compose -f docker-compose.yml -f docker-compose.arm64.yml build camera
    else
        echo "ERROR: neither 'docker compose' nor 'docker-compose' is available." >&2
        exit 1
    fi
else
    echo "[INFO] Image already exists. Skipping build."
fi

if docker compose version >/dev/null 2>&1; then
    docker compose -f docker-compose.yml -f docker-compose.arm64.yml run --rm -T camera /bin/bash -lc '/ws/run.sh'
elif command -v docker-compose >/dev/null 2>&1; then
    docker-compose -f docker-compose.yml -f docker-compose.arm64.yml run --rm -T camera /bin/bash -lc '/ws/run.sh'
else
    echo "ERROR: neither 'docker compose' nor 'docker-compose' is available." >&2
    exit 1
fi
