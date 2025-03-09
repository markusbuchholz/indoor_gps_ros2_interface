#!/usr/bin/env bash

# X11 authorization
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]; then
    xauth_list=$(xauth nlist $DISPLAY)
    xauth_list=$(sed -e 's/^..../ffff/' <<< "$xauth_list")
    if [ ! -z "$xauth_list" ]; then
        echo "$xauth_list" | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

# Local workspace directory
local_workspace="/home/markus/underwater/ubuntu_16/src"



# Set Cyclone DDS environment variables
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/home/ros2_jazzy/cyclonedds.xml
export ROS_DOMAIN_ID=0 

# Running the Docker container with new volume mounts and environment variables
docker run -it \
    --rm \
    --name marvelmind \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION \
    -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
    -v "$XAUTH:$XAUTH" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "/etc/localtime:/etc/localtime:ro" \
    -v "/dev/input:/dev/input" \
    -v "$local_workspace:/home/src" \
    --privileged \
    --security-opt seccomp=unconfined \
    --gpus all \
    marvelmind:latest
