#include "ros/ros.h"
#include "std_msgs/String.h"
#include <gsplines/BasisLegendre.hpp>
#include <gsplines/ipopt_solver.hpp>

int main(int argc, char **argv) {

  ros::init(argc, argv, "my_node_name", ros::init_options::AnonymousName);

  ros::NodeHandle n;

  double exec_time = 10;

  std::size_t number_of_wp = 3;
  std::size_t codom_dim = 7;

  Eigen::MatrixXd wp(Eigen::MatrixXd::Random(number_of_wp, codom_dim));

  gsplines::GSpline trj = gsplines_opt::optimal_sobolev_norm(
      wp, gsplines::basis::BasisLegendre(6), {{1.0, 3}}, exec_time);

  while (ros::ok()) {
  }

  return 0;
}
