#include <boost/format.hpp>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <gsplines_follow_trajectory/follow_joint_trajectory_action_wrapper.hpp>
#include <gsplines_msgs/FollowJointGSplineResult.h>
#include <gsplines_ros/gsplines_ros.hpp>

namespace gsplines_follow_trajectory {
const std::string LOGNAME("GSplineFollowJointTrajectoryActionWrapper");

FollowJointTrajectoryActionWrapper::FollowJointTrajectoryActionWrapper(
    const std::string &_gspline_action_name, const std::string &_fjta_name,
    double _control_step)
    : nh_(), nh_prv_("~"), gspline_action_name_(_gspline_action_name),
      fjta_name_(_fjta_name), control_step_(_control_step),
      action_server_(std::make_unique<actionlib::SimpleActionServer<
                         gsplines_msgs::FollowJointGSplineAction>>(
          nh_, _fjta_name + "/" + gspline_action_name_, false)),
      action_client_(std::make_unique<actionlib::SimpleActionClient<
                         control_msgs::FollowJointTrajectoryAction>>(
          nh_, _fjta_name + "/follow_joint_trajectory", true)) {

  action_server_->registerPreemptCallback(boost::bind(
      &FollowJointTrajectoryActionWrapper::prehemption_action, this));

  action_server_->registerGoalCallback(
      boost::bind(&FollowJointTrajectoryActionWrapper::action_callback, this));

  std::string absolute_namespace = nh_.getNamespace();
  if (absolute_namespace !=
      "/") {                   // Check if we are not in the global namespace.
    absolute_namespace += "/"; // Ensure there is a trailing slash.
  }

  ROS_INFO_STREAM_NAMED(LOGNAME, "Waiting for action "
                                     << absolute_namespace + _fjta_name +
                                            "/follow_joint_trajectory");

  if (action_client_->waitForServer(ros::Duration(60.0))) {
    ROS_INFO("Starting action server %s", gspline_action_name_.c_str());
    action_server_->start();
    ROS_INFO_NAMED(LOGNAME, "Action found.");
  } else {
    ROS_ERROR_STREAM_NAMED(LOGNAME,
                           "Could not find action "
                               << _fjta_name + "/follow_joint_trajectory");
  }
}

void FollowJointTrajectoryActionWrapper::action_callback() {
  // ros::Rate(1).sleep();
  // action_server_->registerGoalCallback();

  ROS_INFO("callback");
  const gsplines_msgs::FollowJointGSplineGoalConstPtr goal =
      action_server_->acceptNewGoal();

  desired_motion_start_time_ = goal->gspline.header.stamp;

  if (not goal) {
    ROS_ERROR("No goal available");
    return;
  }
  forward_goal(goal);
}

void FollowJointTrajectoryActionWrapper::forward_goal(
    const gsplines_msgs::FollowJointGSplineGoalConstPtr &_goal) {

  control_msgs::FollowJointTrajectoryGoal goal_to_forward =
      gsplines_ros::follow_joint_gspline_goal_to_follow_joint_trajectory_goal(
          *_goal, ros::Duration(control_step_));

  action_client_->sendGoal(
      goal_to_forward,
      boost::bind(&FollowJointTrajectoryActionWrapper::done_action, this, _1,
                  _2),
      boost::bind(&FollowJointTrajectoryActionWrapper::active_action, this),
      boost::bind(&FollowJointTrajectoryActionWrapper::feedback_action, this,
                  _1));
}

void FollowJointTrajectoryActionWrapper::feedback_repeater_method(
    const gsplines_msgs::FollowJointGSplineFeedbackConstPtr _msg) {
  action_server_->publishFeedback(_msg);
}

void FollowJointTrajectoryActionWrapper::prehemption_action() {

  action_client_->cancelGoal();
}

void FollowJointTrajectoryActionWrapper::done_action(
    const actionlib::SimpleClientGoalState &state,
    const control_msgs::FollowJointTrajectoryResultConstPtr &_result) {

  gsplines_msgs::FollowJointGSplineResult result = gsplines_ros::
      follow_joint_trajectory_result_to_follow_joint_gspline_result(*_result);

  char text_pos_error[100];
  char text_time_error[100];
  std::sprintf(text_pos_error, "%+14.7e", instant_position_error_inf_norm_);

  switch (state.state_) {
  case actionlib::SimpleClientGoalState::ABORTED:
    action_server_->setAborted(result, state.text_ + "(aborted err = " +
                                           std::string(text_pos_error) + ")");
    break;
  case actionlib::SimpleClientGoalState::REJECTED:
    action_server_->setAborted(result, state.text_ + "(rejected)");
    break;

  case actionlib::SimpleClientGoalState::SUCCEEDED:
    action_server_->setSucceeded(result, state.text_ + "(succeeded)");
    break;

  case actionlib::SimpleClientGoalState::RECALLED:
  case actionlib::SimpleClientGoalState::PREEMPTED:
    action_server_->setPreempted();
    break;
  default:
    break;
  }
  /*
  action_server_->setAborted();
  */
}

void FollowJointTrajectoryActionWrapper::active_action() {}

void FollowJointTrajectoryActionWrapper::feedback_action(
    const control_msgs::FollowJointTrajectoryFeedbackConstPtr &_msg) {

  gsplines_msgs::FollowJointGSplineFeedback result = gsplines_ros::
      follow_joint_trajectory_feedback_to_follow_joint_gspline_feedback(*_msg);

  action_server_->publishFeedback(result);
  instant_position_error_inf_norm_ =
      Eigen::Map<const Eigen::ArrayXd>(_msg->error.positions.data(),
                                       _msg->error.positions.size())
          .abs()
          .maxCoeff();

  //_msg->error.time_from_start.toSec();
}

} // namespace gsplines_follow_trajectory
