
#ifndef GSPLINES_TOOLS
#define GSPLINES_TOOLS
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <gsplines/Basis.hpp>
#include <gsplines/GSpline.hpp>
#include <gsplines_msgs/GSpline.h>
#include <gsplines_msgs/JointGSpline.h>
#include <memory>
#include <string>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace gsplines_ros {

gsplines::GSpline msg_to_gspline(const gsplines_msgs::GSpline &_msg);

gsplines_msgs::GSpline gspline_to_msg(const gsplines::GSpline &_gspline);

gsplines_msgs::JointGSpline
gspline_to_joint_gspline_msg(const gsplines::GSpline &_gspline,
                             const std::vector<std::string> &_joint_names);

trajectory_msgs::JointTrajectory
gspline_to_joint_trajectory_msgs(const gsplines::GSpline &_trj,
                                 const std::vector<std::string> &_joint_names,
                                 const ros::Duration &_rate);

trajectory_msgs::JointTrajectory gspline_msg_to_joint_trajectory_msgs(
    const gsplines_msgs::GSpline &_trj,
    const std::vector<std::string> &_joint_names, const ros::Duration &_rate);

trajectory_msgs::JointTrajectory joint_gspline_msg_to_joint_trajectory_msgs(
    const gsplines_msgs::JointGSpline &_trj, const ros::Duration &_rate);

control_msgs::FollowJointTrajectoryGoal gspline_to_follow_joint_trajectory_goal(
    const gsplines::GSpline &_trj, const std::vector<std::string> &_joint_names,
    const ros::Duration &_rate);

control_msgs::FollowJointTrajectoryGoal
gspline_msgs_to_follow_joint_trajectory_goal(
    const gsplines_msgs::GSpline &_trj,
    const std::vector<std::string> &_joint_names, const ros::Duration &_rate);

control_msgs::FollowJointTrajectoryGoal
joint_gspline_msgs_to_follow_joint_trajectory_goal(
    const gsplines_msgs::JointGSpline &_trj, const ros::Duration &_rate);
} // namespace gsplines_ros

#endif /* ifndef GSPLINES_TOOLS                                                \
#include<gsplines>                                                             \
 */
