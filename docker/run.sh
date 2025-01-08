xhost +
docker run --rm -it --ipc=host --net=host --privileged \
    --env="DISPLAY" \
    --volume="/etc/localtime:/etc/localtime:ro" \
    dongjae0107/lt-mapper:latest
xhost -