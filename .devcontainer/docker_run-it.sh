#!/bin/bash

docker rm ubuntu

docker run -it \
  --user robo25 \
  --name ubuntu \
  --network=host \
  --ipc=host \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --privileged \
  --mount type=bind,source=/home/robo24/robo25ws,target=/home/robo25/robo25ws \
  ubuntu-osrf-jammy


# --privileged for now   --device=/dev/ttyACM0 \
# {"str":50}