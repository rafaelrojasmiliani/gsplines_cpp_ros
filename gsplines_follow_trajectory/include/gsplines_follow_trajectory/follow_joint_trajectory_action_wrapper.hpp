#ifndef WRAPPER_H
#define WRAPPER_H

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <control_msgs/FollowJointTrajectoryResult.h>

#include <gsplines_msgs/FollowJointGSplineAction.h>
#include <gsplines_msgs/FollowJointGSplineFeedback.h>
#include <gsplines_msgs/FollowJointGSplineGoal.h>
#include <gsplines_msgs/FollowJointGSplineResult.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/time.h>

#include <memory>
#include <string>

namespace gsplines_follow_trajectory {

/// This is a relay/repeater node which translates FollowJointGSplineAction
/// into a FollowJointTrajectoryAction.
class FollowJointTrajectoryActionWrapper {
protected:
  /// namespace where the node runs
  ros::NodeHandle nh_;
  /// private namespace
  ros::NodeHandle nh_prv_;

private:
  /// gspline feedback message buffer
  gsplines_msgs::FollowJointGSplineFeedback feedback_;

  /// gspline result message buffer
  gsplines_msgs::FollowJointGSplineResult result_;

  std::string gspline_action_name_;
  std::string fjta_name_; // follow joint trajectory action name

  ros::Subscriber feedback_subscriber_;
  ros::Publisher feedback_repeater_;

  double instant_position_error_inf_norm_ = 0.0;

protected:
  std::unique_ptr<
      actionlib::SimpleActionServer<gsplines_msgs::FollowJointGSplineAction>>
      action_server_;

  std::unique_ptr<
      actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>
      action_client_;

  ros::Time desired_motion_start_time_;

public:
  // FollowJointTrajectoryActionWrapper(
  //     const FollowJointTrajectoryActionWrapper &) = delete;
  // FollowJointTrajectoryActionWrapper &
  // operator=(const FollowJointTrajectoryActionWrapper &) = delete;

  /**
   * @brief Constructor of the wappre
   *
   * @param _gspline_action_name name of the action exposed by the action server
   * @param _fjta_name name of the action client
   * @param _control_step time step of the discretization of the gspline when
   * transformed into a follow joint trajectory
   */
  FollowJointTrajectoryActionWrapper(std::string _gspline_action_name,
                                     std::string _fjta_name,
                                     double _control_step);

  /// Default destructor
  virtual ~FollowJointTrajectoryActionWrapper();

  /**
   * @brief Non blocking. Transform a gspline goal into a follow joint
   * trajectory action goal and send it to the target client
   *
   * @param goal desired gspline goal
   */
  void forward_goal(const gsplines_msgs::FollowJointGSplineGoal &goal);

  /**
   * @brief Blocking. Transform a gspline goal into a follow joint trajectory
   * action goal and send it to the target client
   *
   * @param goal desired gspline goal
   */
  void forward_goal_and_wait(
      const gsplines_msgs::FollowJointGSplineGoalConstPtr &goal);

  /**
   * @brief Action callback of the action server of this instance. Here the goal
   * forwarding is implemented
   *
   */
  virtual void action_callback();

  void feedback_repeater_method(
      const gsplines_msgs::FollowJointGSplineFeedbackConstPtr &_msg);

  /**
   * @brief Action executed on preemption request
   *
   */
  virtual void preemption_action();

  /**
   * @brief Action executed when the task is concluded
   *
   * @param state Goal state
   * @param _result result of the joint trajectory action
   */
  virtual void
  done_action(const actionlib::SimpleClientGoalState &state, // NOLINT
              const control_msgs::FollowJointTrajectoryResultConstPtr &_result);

  /**
   * @brief Action executed with the target action client starts its goal
   *
   */
  virtual void active_action();

  /**
   * @brief Action performed when the target action client sends a feedback
   *
   * @paramD _msg feedback from the target client.
   */
  virtual void feedback_action(
      const control_msgs::FollowJointTrajectoryFeedbackConstPtr &_msg);

  void forward_state(const actionlib::SimpleClientGoalState &state);

  /**
   * @brief Returns the control step. The discretization used to gerenate the
   * follow joint trajectory goal from the gspline messge.
   *
   * @return the control step.
   */
  [[nodiscard]] double get_control_step();

  /**
   * @brief Get the  time when the goal trajectory should start.
   *
   */
  [[nodiscard]] const ros::Time &get_desired_motion_start_time() const {
    return desired_motion_start_time_;
  }

protected:
  class Impl;
  std::unique_ptr<Impl> m_impl;
};

} // namespace gsplines_follow_trajectory

#endif /* WRAPPER_H */
