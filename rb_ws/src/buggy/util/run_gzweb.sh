#!/bin/bash

# ENV setup, cd to gzweb install dir
DIR=/root/gzweb
cd $DIR

# Regenerate model database
mkdir -p $DIR/http/client/assets
export GAZEBO_MODEL_PATH=/rb_ws/src/buggy/sim_models
./get_local_models.py $DIR/http/client/assets
./webify_models_v2.py $DIR/http/client/assets

# Start the server
echo "Gazebo Web Client setup complete! Navigate to http://localhost:3000 in your web browser."
npm start -p 3000