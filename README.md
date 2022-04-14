# General Splines Library in ROS

Implementation of the [gsplines](https://github.com/rafaelrojasmiliani/gsplines_cpp) library in ROS

- Piecewise polynomial curves representation
- Automatic **exact** (and fast) differentiation of generalized splines
- Algebraic operations: inner product, norms, addition, multiplication, composition and concatenation of curves (allows only when it has mathematical sense).
- Optimization with waypoint (via-point) constraints: minimum jerk, snap, crank, etc.
- Implements piece-wise Lagrange polynomials at Gauss-Lobatto points.
- MoveIt implementation [here](https://github.com/rafaelrojasmiliani/gsplines_moveit)

# Installation

## In ubuntu with ROS noetic

1. Install dependencies
```
apt-get update
apt-get install ros-noetic-ifopt python3-matplotlib
wget https://github.com/rafaelrojasmiliani/gsplines_cpp/releases/download/package/gsplines-0.0.1-amd64.deb
dpkg -i gsplines-0.0.1-amd64.deb
```

2. Download this repo and `catkin build`
