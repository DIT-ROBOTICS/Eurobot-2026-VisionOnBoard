#!/bin/bash

cd docker/ || { echo "Directory 'docker/' not found"; exit 1; }

# Take down running containers and networks first
docker compose down

# Build and start fresh
docker compose build && docker compose up -d