#!/bin/bash
# Run to spin up docker containers and set aliases
docker compose -f docker-compose-no-gpu.yml down # kill all running containers
docker compose -f docker-compose-no-gpu.yml build
docker compose -f docker-compose-no-gpu.yml --env-file .env.dev up -d

sleep 0.5

echo "DEBUG: Buggy Docker Container Up!"
echo "Run docker_exec in order to go into the Docker container"
