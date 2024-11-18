xhost +
docker run --rm -it --ipc=host --net=host --privileged \
    --env="DISPLAY" \
    --volume="/etc/localtime:/etc/localtime:ro" \
    --volume="/media/ori/Extreme SSD/oxford_spires/sequences:/dataset" \
    --volume="/home/ori/Downloads/sc_lio_sam:/sc_lio_sam" \
    dongjae0107/lt-mapper:latest
xhost -