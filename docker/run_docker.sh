#! /bin/bash


#run the docker container
docker container run --runtime=nvidia -it -e DISPLAY  --rm --net=host --env="QT_X11_NO_MITSHM=1" -v /tmp/.X11-unix:/tmp/.X11-unix platoon_test bash