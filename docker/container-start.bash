main(){

    scriptdir=$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)
    if [ "$scriptdir" != "$(pwd)" ]; then
      echo "this script must be executed from $scriptdir".
      exit 1
    fi


    if lspci | grep -qi "vga .*nvidia" && \
        docker -D info 2>/dev/null | grep -qi "runtimes.* nvidia"; then
        NVIDIA_DOCKER_PARAMS="\
            --runtime=nvidia \
            --env=NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all} \
            --env=NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}all"
    else
        NVIDIA_DOCKER_PARAMS=""
    fi

    XAUTH=/tmp/.docker.xauth
    touch $XAUTH
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -


    docker run -it \
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        ${NVIDIA_DOCKER_PARAMS} \
        --env="XAUTHORITY=$XAUTH" \
        --volume="$XAUTH:$XAUTH" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --env="ROS_MASTER_URI=http://127.0.0.1:11311" \
        --env="ROS_MASTER_IP=127.0.0.1" \
        --env="ROS_IP=127.0.0.1" \
        --volume $(pwd)/../:/catkinws/src/ \
        --user $(id -u):$(id -g) \
        "gsplines_cpp_ros" bash
}

main
