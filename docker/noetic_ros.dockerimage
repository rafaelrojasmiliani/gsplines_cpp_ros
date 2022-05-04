FROM rafa606/vim-ros-noetic-gsplines
SHELL ["bash", "-c"]
RUN mkdir -p /aux_ws/src \
    && git clone \
        https://github.com/rafaelrojasmiliani/gsplines_cpp_ros.git \
        /aux_ws/src/gsplines_ros \
    && cd /aux_ws \
    && source /opt/ros/noetic/setup.bash \
    && cd /aux_ws \
    && catkin config \
        --install --install-space /opt/ros/noetic/ \
        --extend /opt/ros/noetic/ \
    && catkin build
