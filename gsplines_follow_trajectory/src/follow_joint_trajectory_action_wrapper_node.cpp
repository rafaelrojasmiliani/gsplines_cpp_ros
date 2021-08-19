#include <ros/ros.h>

#include <gsplines_follow_trajectory/follow_joint_trajectory_action_wrapper.hpp>

int main(int argc, char **argv) {

  ros::init(argc, argv, "robot_state_publisher");
  ros::NodeHandle node;

  gsplines_follow_trajectory::FollowJointTrajectoryActionWrapper wrapper(
      "gspline_grapper", "pos_joint_traj_controller/");

  ros::spin();

  return 0;
}
