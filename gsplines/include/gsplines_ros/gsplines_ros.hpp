
#ifndef GSPLINES_TOOLS
#define GSPLINES_TOOLS
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <gsplines/Basis.hpp>
#include <gsplines/GSpline.hpp>
#include <gsplines_msgs/GSpline.h>
#include <memory>
#include <string>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace gsplines {

GSpline msg_to_gspline(const gsplines_msgs::GSpline &_msg);

gsplines_msgs::GSpline gspline_to_msg(const GSpline &_gspline);

trajectory_msgs::JointTrajectory gspline_to_joint_trajectory_message(
    GSpline _trj, std::vector<std::string> _joint_names, ros::Duration &_rate);

control_msgs::FollowJointTrajectoryGoal gspline_to_follow_joint_trajectory_goal(
    GSpline _trj, std::vector<std::string> _joint_names, ros::Duration &_rate);

} // namespace gsplines

#endif /* ifndef GSPLINES_TOOLS                                                \
#include<gsplines>                                                             \
 */
