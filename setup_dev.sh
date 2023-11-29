#!/bin/bash

no_gpu=false
force_gpu=false

usage() {
  echo "Usage: $0 [options]"
  echo "Options:"
  echo " --no-gpu        Disable CUDA support. Required to run on systems without Nvidia Container Toolkit."
  echo " --force-gpu     Force a GPU build even if Nvidia Container Toolkit is not detected"
}

while [ $# -gt 0 ]; do
  case $1 in
    -h | --help)
      usage
      exit 0
      ;;
    --no-gpu)
      no_gpu=true
      ;;
    --force-gpu)
      force_gpu=true
      ;;
    *)
      echo "Invalid option: $1" >&2
      usage
      exit 1
      ;;
  esac
  shift
done

if $no_gpu && $force_gpu
then
  echo -e "\033[0;31mOptions --no-gpu and --force-gpu conflict. Use at most one of them.\033[0m"
  exit 1
fi

if ! $no_gpu && ! $force_gpu && ! command -v nvidia-ctk &> /dev/null
then
  echo -e "\033[0;31mNvidia Container Toolkit was not found.\033[0m"
  echo -e "\033[0;31mRun with --force-gpu to build with GPU.\033[0m"
  echo -e "\033[0;31mRun with --no-gpu to silence this message.\033[0m"
  echo -e "\033[0;31mContinuing with no GPU...\033[0m"
  no_gpu=true
fi

#####################
# Actual logic here #
#####################

if $no_gpu
then
  dockerfile="docker-compose-no-gpu.yml"
else
  dockerfile="docker-compose-gpu.yml"
fi

echo "Killing old development containers..."
docker compose -f docker-compose-no-gpu.yml down # kill old containers
docker compose -f docker-compose-gpu.yml down # kill old containers

echo "Building containers..."
docker compose -f $dockerfile build

echo "Starting containers..."
docker compose -f $dockerfile --env-file .env.dev up -d

sleep 0.5

echo "DEBUG: Buggy Docker Container Up!"
echo "Run docker_exec in order to go into the Docker container"
