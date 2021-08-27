#ifndef WRAPPER_H
#define WRAPPER_H

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <gsplines_msgs/FollowJointGSplineAction.h>
#include <ros/ros.h>

namespace gsplines_follow_trajectory {

class FollowJointTrajectoryActionWrapper {
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_prv_;

  gsplines_msgs::FollowJointGSplineFeedback feedback_;
  gsplines_msgs::FollowJointGSplineResult result_;
  std::string name_;
  std::string fjta_name_; // follow joint trajectory action name

  ros::Subscriber feedback_subscriber_;
  ros::Publisher feedback_repeater_;

  double control_step_;
  std::string target_action_ns_;
  double instant_position_error_inf_norm_;
  double time_from_start_error_;

protected:
  std::unique_ptr<
      actionlib::SimpleActionServer<gsplines_msgs::FollowJointGSplineAction>>
      action_server_;

  std::unique_ptr<
      actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>
      action_client_;

public:
  FollowJointTrajectoryActionWrapper(
      const FollowJointTrajectoryActionWrapper &) = delete;
  FollowJointTrajectoryActionWrapper &
  operator=(const FollowJointTrajectoryActionWrapper &) = delete;

  FollowJointTrajectoryActionWrapper(const std::string &_name,
                                     const std::string &_fjta_name,
                                     double _control_step);

  virtual ~FollowJointTrajectoryActionWrapper() = default;

  void forward_goal(const gsplines_msgs::FollowJointGSplineGoalConstPtr &goal);
  void forward_goal_and_wait(
      const gsplines_msgs::FollowJointGSplineGoalConstPtr &goal);
  virtual void action_callback();

  void feedback_repeater_method(
      const gsplines_msgs::FollowJointGSplineFeedbackConstPtr _msg);
  virtual void prehemption_action();

  virtual void
  done_action(const actionlib::SimpleClientGoalState &state,
              const control_msgs::FollowJointTrajectoryResultConstPtr &_result);

  virtual void active_action();
  virtual void feedback_action(
      const control_msgs::FollowJointTrajectoryFeedbackConstPtr &_result);

  void forward_state(const actionlib::SimpleClientGoalState &state);

  double get_control_step() const { return control_step_; }
};

} // namespace gsplines_follow_trajectory

#endif /* WRAPPER_H */
