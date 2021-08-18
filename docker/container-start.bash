main(){

    scriptdir=$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)
    if [ "$scriptdir" != "$(pwd)" ]; then
      echo "this script must be executed from $scriptdir".
      exit 1
    fi

    docker run -it \
        --env="DISPLAY" \
        --env="ROS_MASTER_URI=http://127.0.0.1:11311" \
        --env="ROS_MASTER_IP=127.0.0.1" \
        --env="ROS_IP=127.0.0.1" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume $(pwd)/../:/catkinws/src/ \
        --user $(id -u):$(id -g) \
        "gsplines_cpp_ros" bash
}

main
