#include <eigen3/Eigen/Core>
#include <gsplines/GSpline.hpp>
#include <gsplines_ros/gsplines_ros.hpp>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <pybind11/stl.h>

namespace gsplines_ros {
PYBIND11_MODULE(pygsplines_ros, m) {
  m.def("msg_to_gspline", msg_to_gspline);

  m.def("gspline_to_msg", gspline_to_msg);

  m.def("gspline_to_joint_gspline_msg", gspline_to_joint_gspline_msg);

  m.def("gspline_to_joint_trajectory_msgs", gspline_to_joint_trajectory_msgs);

  m.def("gspline_msg_to_joint_trajectory_msgs",
        gspline_msg_to_joint_trajectory_msgs);

  m.def("joint_gspline_msg_to_joint_trajectory_msgs",
        joint_gspline_msg_to_joint_trajectory_msgs);

  m.def("gspline_to_follow_joint_trajectory_goal",
        gspline_to_follow_joint_trajectory_goal);

  m.def("gspline_msgs_to_follow_joint_trajectory_goal",
        gspline_msgs_to_follow_joint_trajectory_goal);

  m.def("joint_gspline_msgs_to_follow_joint_trajectory_goal",
        joint_gspline_msgs_to_follow_joint_trajectory_goal);
} // namespace gsplines_ros
} // namespace gsplines_ros
