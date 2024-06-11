
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryResult.h>

#include <gsplines_follow_trajectory/follow_joint_trajectory_action_wrapper.hpp>

#include <gsplines_msgs/FollowJointGSplineAction.h>
#include <gsplines_msgs/FollowJointGSplineFeedback.h>
#include <gsplines_msgs/FollowJointGSplineGoal.h>
#include <gsplines_msgs/FollowJointGSplineResult.h>

#include <gsplines_follow_trajectory/FollowJointTrajectoryActionWrapperDynamicReconfigureConfig.h>
#include <gsplines_ros/gsplines_ros.hpp>

#include <ros/duration.h>
#include <ros/node_handle.h>

#include <dynamic_reconfigure/server.h>

#include <memory>
#include <string>
#include <utility>

namespace gsplines_follow_trajectory {

const std::string LOGNAME("GSplineFollowJointTrajectoryActionWrapper");

class FollowJointTrajectoryActionWrapper::Impl {
public:
  double control_step = 0.01;
  using ConfigType = FollowJointTrajectoryActionWrapperDynamicReconfigureConfig;

  dynamic_reconfigure::Server<ConfigType> server_;

  ros::NodeHandle nh_prv_{"~"};

  Impl(double _control_step)
      : server_(ros::NodeHandle("~")), control_step(_control_step) {

    server_.setCallback([this](ConfigType &_cfg, uint32_t _level) {
      this->set_parameters(_cfg, _level);
    });

    ConfigType config;
    config.control_step = _control_step;

    server_.updateConfig(config);
  }

  void set_parameters(ConfigType &_cfg, uint32_t level) {
    (void)level;

    control_step = _cfg.control_step;
  }
};

FollowJointTrajectoryActionWrapper::FollowJointTrajectoryActionWrapper(
    std::string _gspline_action_name, std::string _fjta_name,
    double _control_step)
    : nh_prv_("~"), // init private namespace
      gspline_action_name_(
          std::move(_gspline_action_name)), // gspline action name
      fjta_name_(std::move(_fjta_name)),    // FollowJointTrajectoryAction name
                                            // FollowJointTrajectoryGoal
      action_server_(std::make_unique<actionlib::SimpleActionServer< // NOLINT
                         gsplines_msgs::FollowJointGSplineAction>>(
          nh_, fjta_name_ + "/" + gspline_action_name_, false)),
      action_client_(std::make_unique<actionlib::SimpleActionClient< // NOLINT
                         control_msgs::FollowJointTrajectoryAction>>(
          nh_, fjta_name_ + "/follow_joint_trajectory", true)),
      m_impl(new Impl(_control_step)) {

  action_server_->registerPreemptCallback([this] { preemption_action(); });

  action_server_->registerGoalCallback([this] { action_callback(); });

  std::string absolute_namespace = nh_.getNamespace();
  if (absolute_namespace !=
      "/") {                   // Check if we are not in the global namespace.
    absolute_namespace += "/"; // Ensure there is a trailing slash.
  }

  ROS_INFO_STREAM_NAMED(LOGNAME, "Waiting for action " // NOLINT
                                     << absolute_namespace + fjta_name_ +
                                            "/follow_joint_trajectory");

  if (action_client_->waitForServer(ros::Duration(60.0))) {
    ROS_INFO("Starting action server %s",   // NOLINT
             gspline_action_name_.c_str()); // NOLINT
    action_server_->start();
    ROS_INFO_NAMED(LOGNAME, "Action found."); // NOLINT
  } else {
    ROS_ERROR_STREAM_NAMED(LOGNAME, // NOLINT
                           "Could not find action "
                               << _fjta_name + "/follow_joint_trajectory");
  }
}

double FollowJointTrajectoryActionWrapper::get_control_step() {
  return m_impl->control_step;
}

FollowJointTrajectoryActionWrapper::~FollowJointTrajectoryActionWrapper() =
    default;

void FollowJointTrajectoryActionWrapper::action_callback() {

  const auto &goal = action_server_->acceptNewGoal();

  if (!goal) {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "No goal available"); // NOLINT
    action_server_->setAborted();
    return;
  }
  desired_motion_start_time_ = goal->gspline.header.stamp;
  forward_goal(*goal);
}

void FollowJointTrajectoryActionWrapper::forward_goal(
    const gsplines_msgs::FollowJointGSplineGoal &_goal) {

  control_msgs::FollowJointTrajectoryGoal goal_to_forward =
      gsplines_ros::follow_joint_gspline_goal_to_follow_joint_trajectory_goal(
          _goal, ros::Duration(m_impl->control_step));

  action_client_->sendGoal(
      goal_to_forward,
      [this](auto &&PH1, auto &&PH2) {
        done_action(std::forward<decltype(PH1)>(PH1),
                    std::forward<decltype(PH2)>(PH2));
      },
      [this] { active_action(); },
      [this](auto &&PH1) {
        feedback_action(std::forward<decltype(PH1)>(PH1));
      });

  ROS_INFO_STREAM_NAMED(LOGNAME, "Goal Forwarded");
}

void FollowJointTrajectoryActionWrapper::feedback_repeater_method(
    const gsplines_msgs::FollowJointGSplineFeedbackConstPtr &_msg) {
  action_server_->publishFeedback(_msg);
}

void FollowJointTrajectoryActionWrapper::preemption_action() {

  action_client_->cancelGoal();
}

void FollowJointTrajectoryActionWrapper::done_action(
    const actionlib::SimpleClientGoalState &state, // NOLINT
    const control_msgs::FollowJointTrajectoryResultConstPtr &_result) {

  gsplines_msgs::FollowJointGSplineResult result = gsplines_ros::
      follow_joint_trajectory_result_to_follow_joint_gspline_result(*_result);

  char text_pos_error[100];                       // NOLINT
  std::sprintf(text_pos_error, "%+14.7e",         // NOLINT
               instant_position_error_inf_norm_); // NOLINT

  switch (state.state_) {
  case actionlib::SimpleClientGoalState::ABORTED:
    action_server_->setAborted(
        result, state.text_ +
                    "(aborted err = " + std::string(text_pos_error) + // NOLINT
                    ")");                                             // NOLINT
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
}

void FollowJointTrajectoryActionWrapper::active_action() {}

void FollowJointTrajectoryActionWrapper::feedback_action(
    const control_msgs::FollowJointTrajectoryFeedbackConstPtr &_msg) {

  gsplines_msgs::FollowJointGSplineFeedback result = gsplines_ros::
      follow_joint_trajectory_feedback_to_follow_joint_gspline_feedback(*_msg);

  action_server_->publishFeedback(result);
  instant_position_error_inf_norm_ =
      Eigen::Map<const Eigen::ArrayXd>(
          _msg->error.positions.data(),
          static_cast<long>(_msg->error.positions.size()))
          .abs()
          .maxCoeff();

  //_msg->error.time_from_start.toSec();
}

} // namespace gsplines_follow_trajectory
