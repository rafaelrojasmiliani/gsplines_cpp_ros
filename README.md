# General Splines Library in ROS

Implementation of the [gsplines](https://github.com/rafaelrojasmiliani/gsplines_cpp) library in ROS

- Piecewise polynomial curves representation
- Automatic **exact** (and fast) differentiation of generalized splines
- Algebraic operations: inner product, norms, addition, multiplication, composition and concatenation of curves (allows only when it has mathematical sense).
- Optimization with waypoint (via-point) constraints: minimum jerk, snap, crank, etc.
- Implements piece-wise Lagrange polynomials at Gauss-Lobatto points.
- MoveIt implementation [here](https://github.com/rafaelrojasmiliani/gsplines_moveit)

# Installation

## In ubuntu with ROS

1. Install dependencies
```
apt-get update
apt-get install ros-${ROS_DISTRO}-ifopt python3-matplotlib libgtest-dev cmake libeigen3-dev coinor-libipopt-dev
```
2. Install the GSplines
```
source /opt/ros/${ROS_DISTRO}/setup.bash && git clone --recursive https://github.com/rafaelrojasmiliani/gsplines_cpp.git ~/gsplines && cd ~/gsplines && mkdir build && cd build && cmake .. -DCMAKE_INSTALL_PREFIX=/usr && make && make install \
   && rm -rf ~/gsplines
```

3. Download this repo to your workspace and `catkin build`, or install
