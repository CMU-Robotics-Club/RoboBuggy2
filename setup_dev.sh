#!/bin/bash

no_gpu=false
force_gpu=false
run_automated_testing_docker=false

usage() {
  echo "Usage: $0 [options]"
  echo "Options:"
  echo " --no-gpu        Disable CUDA support. Required to run on systems without Nvidia Container Toolkit."
  echo " --force-gpu     Force a GPU build even if Nvidia Container Toolkit is not detected"
  echo " --run-testing   Run the testing docker container"
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
    --run-testing)
      run_automated_testing_docker=true
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

if $no_gpu && ! $run_automated_testing_docker
then
  dockerfile="docker-compose-no-gpu.yml"
fi

if ! $no_gpu && ! $run_automated_testing_docker
then
  dockerfile="docker-compose-gpu.yml"
fi

if $no_gpu && $run_automated_testing_docker
then
  dockerfile="docker-compose-no-gpu-automated-testing.yml"
fi

if ! $no_gpu && $run_automated_testing_docker
then
  dockerfile="docker-compose-gpu-automated-testing.yml"
fi

echo "Killing old development containers..."
docker compose -f docker-compose-gpu-automated-testing.yml down # kill old containers, this is most complete

# copy python-requirements + cuda-requirements into appropriate folders for docker compose
cp python-requirements.txt docker_auton
mv docker_auton/python-requirements.txt docker_auton/python-requirements_TEMP_DO_NOT_EDIT.txt
cp python-requirements.txt docker_tester
mv docker_tester/python-requirements.txt docker_tester/python-requirements_TEMP_DO_NOT_EDIT.txt
cp cuda-requirements.txt docker_auton
mv docker_auton/cuda-requirements.txt docker_auton/cuda-requirements_TEMP_DO_NOT_EDIT.txt

echo "Building containers..."
docker compose -f $dockerfile build

echo "Starting containers..."
docker compose -f $dockerfile --env-file .env.dev up -d

sleep 0.5

echo "DEBUG: Buggy Docker Container Up!"
echo "Run docker_exec in order to go into the Docker container"
