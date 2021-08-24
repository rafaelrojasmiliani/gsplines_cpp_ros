#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <gsplines_follow_trajectory/follow_joint_trajectory_action_wrapper.hpp>
#include <gsplines_msgs/FollowJointGSplineResult.h>
#include <gsplines_ros/gsplines_ros.hpp>
#include <map>
#include <variant>

namespace gsplines_follow_trajectory {

FollowJointTrajectoryActionWrapper::FollowJointTrajectoryActionWrapper(
    const std::string &_name, const std::string &_fjta_name,
    double _control_step)
    : nh_(), nh_prv_("~"), name_(_name), fjta_name_(_fjta_name),
      action_server_(std::make_unique<actionlib::SimpleActionServer<
                         gsplines_msgs::FollowJointGSplineAction>>(
          nh_, _name,
          boost::bind(&FollowJointTrajectoryActionWrapper::action_callback,
                      this, _1),
          false)),
      action_client_(std::make_unique<actionlib::SimpleActionClient<
                         control_msgs::FollowJointTrajectoryAction>>(
          nh_, _fjta_name + "/follow_joint_trajectory", true)),
      control_step_(_control_step) {

  action_server_->registerPreemptCallback(boost::bind(
      &FollowJointTrajectoryActionWrapper::prehemption_action, this));

  if (action_client_->waitForServer(ros::Duration(60.0))) {
    action_server_->start();
  }
}

void FollowJointTrajectoryActionWrapper::action_callback(
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
                  _1)

  );
  action_client_->getState();

  action_client_->waitForResult();
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

  switch (state.state_) {
  case actionlib::SimpleClientGoalState::ABORTED:
  case actionlib::SimpleClientGoalState::REJECTED:
    action_server_->setAborted(result, state.text_);
    break;

  case actionlib::SimpleClientGoalState::SUCCEEDED:
    action_server_->setSucceeded(result, state.text_);
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
}

} // namespace gsplines_follow_trajectory
