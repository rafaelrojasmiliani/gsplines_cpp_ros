#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <gsplines_follow_trajectory/follow_joint_trajectory_action_wrapper.hpp>
#include <gsplines_ros/gsplines_ros.hpp>

namespace gsplines_follow_trajectory {

FollowJointTrajectoryActionWrapper::FollowJointTrajectoryActionWrapper(
    const std::string &_name, const std::string &_fjta_name)
    : nh_(), nh_prv_("~"), name_(_name), fjta_name_(_fjta_name),
      action_server_(
          nh_, _name,
          boost::bind(&FollowJointTrajectoryActionWrapper::action_callback,
                      this, _1),
          false),
      action_client_(nh_, _fjta_name + "/follow_joint_trajectory", true) {

  action_server_.registerPreemptCallback(boost::bind(
      &FollowJointTrajectoryActionWrapper::prehemption_action, this));

  if (action_client_.waitForServer(ros::Duration(5.0))) {
    action_server_.start();
  }
}
void FollowJointTrajectoryActionWrapper::action_callback(
    const gsplines_msgs::FollowJointGSplineGoalConstPtr &goal) {

  control_msgs::FollowJointTrajectoryGoal goal_to_forward;

  goal_to_forward.goal_time_tolerance = goal->goal_time_tolerance;
  goal_to_forward.goal_tolerance = goal->goal_tolerance;
  goal_to_forward.path_tolerance = goal->path_tolerance;

  goal_to_forward.trajectory =
      gsplines_ros::joint_gspline_msg_to_joint_trajectory_msgs(
          goal->gspline, ros::Duration(3.0));
  action_client_.sendGoal(
      goal_to_forward,
      boost::bind(&FollowJointTrajectoryActionWrapper::done_action, this));
}

void FollowJointTrajectoryActionWrapper::feedback_repeater_method(
    const gsplines_msgs::FollowJointGSplineFeedbackConstPtr _msg) {
  action_server_.publishFeedback(_msg);
}

void FollowJointTrajectoryActionWrapper::prehemption_action() {}
void FollowJointTrajectoryActionWrapper::done_action() {}
void FollowJointTrajectoryActionWrapper::active_action() {}
void FollowJointTrajectoryActionWrapper::feedback_action() {}
} // namespace gsplines_follow_trajectory
