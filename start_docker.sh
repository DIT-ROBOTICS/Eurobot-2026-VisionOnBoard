#!/bin/bash

cd docker/ || { echo "Directory 'docker/' not found"; exit 1; }

# Build AND then Up
docker compose build && docker compose up -d