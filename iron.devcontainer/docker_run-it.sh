#!/bin/bash

docker rm iron

docker run -it \
  --user robo25 \
  --name iron \
  --network=host \
  --ipc=host \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --privileged \
  --mount type=bind,source=/home/robo24/robo25ws,target=/home/robo25/robo25ws \
  iron
