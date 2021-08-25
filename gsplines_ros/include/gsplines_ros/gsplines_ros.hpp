
#ifndef GSPLINES_TOOLS
#define GSPLINES_TOOLS
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryResult.h>
#include <gsplines/Basis.hpp>
#include <gsplines/Functions/FunctionExpression.hpp>
#include <gsplines/GSpline.hpp>
#include <gsplines_msgs/FollowJointGSplineFeedback.h>
#include <gsplines_msgs/FollowJointGSplineGoal.h>
#include <gsplines_msgs/FollowJointGSplineResult.h>
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

trajectory_msgs::JointTrajectory function_expression_to_joint_trajectory_msg(
    const gsplines::functions::FunctionExpression &_trj,
    const std::vector<std::string> &_joint_names, const ros::Duration &_rate);

trajectory_msgs::JointTrajectory gspline_msg_to_joint_trajectory_msgs(
    const gsplines_msgs::GSpline &_trj,
    const std::vector<std::string> &_joint_names, const ros::Duration &_rate);

trajectory_msgs::JointTrajectory joint_gspline_msg_to_joint_trajectory_msgs(
    const gsplines_msgs::JointGSpline &_trj, const ros::Duration &_rate);

control_msgs::FollowJointTrajectoryGoal gspline_to_follow_joint_trajectory_goal(
    const gsplines::GSpline &_trj, const std::vector<std::string> &_joint_names,
    const ros::Duration &_rate);

control_msgs::FollowJointTrajectoryGoal
function_expression_to_follow_joint_trajectory_goal(
    const gsplines::functions::FunctionExpression &_trj,
    const std::vector<std::string> &_joint_names, const ros::Duration &_rate);

control_msgs::FollowJointTrajectoryGoal
gspline_msgs_to_follow_joint_trajectory_goal(
    const gsplines_msgs::GSpline &_trj,
    const std::vector<std::string> &_joint_names, const ros::Duration &_rate);

control_msgs::FollowJointTrajectoryGoal
joint_gspline_msgs_to_follow_joint_trajectory_goal(
    const gsplines_msgs::JointGSpline &_trj, const ros::Duration &_rate);

gsplines_msgs::FollowJointGSplineFeedback
follow_joint_trajectory_feedback_to_follow_joint_gspline_feedback(
    const control_msgs::FollowJointTrajectoryFeedback &_msg);

control_msgs::FollowJointTrajectoryGoal
follow_joint_gspline_goal_to_follow_joint_trajectory_goal(
    const gsplines_msgs::FollowJointGSplineGoal &_msg,
    const ros::Duration &_control_step);

control_msgs::FollowJointTrajectoryResult
follow_joint_gspline_result_to_follow_joint_trajectory_result(
    const gsplines_msgs::FollowJointGSplineResult &_msg);

gsplines_msgs::FollowJointGSplineResult
follow_joint_trajectory_result_to_follow_joint_gspline_result(
    const control_msgs::FollowJointTrajectoryResult &_msg);
} // namespace gsplines_ros

#endif
