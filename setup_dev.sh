#!/bin/bash
# Run to spin up docker containers and set aliases
docker compose down # kill all running containers
docker compose build
docker compose --env-file .env.dev up &

sleep 0.5

echo "DEBUG: Buggy Docker Container Up!"
echo "Run docker_exec in order to go into the Docker container"