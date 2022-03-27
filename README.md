# General Splines Library in ROS

Implementation of the [gsplines](https://github.com/rafaelrojasmiliani/gsplines_cpp) library in ROS

- Piecewise polynomial curves representation
- Automatic **exact** (and fast) differentiation of generalized splines
- Algebraic operations: inner product, norms, addition, multiplication, composition and concatenation of curves (allows only when it has mathematical sense).
- Optimization with waypoint (via-point) constraints: minimum jerk, snap, crank, etc.
- Implements piece-wise Lagrange polynomials at Gauss-Lobatto points.
- MoveIt implementation [here](https://github.com/rafaelrojasmiliani/gsplines_moveit)

