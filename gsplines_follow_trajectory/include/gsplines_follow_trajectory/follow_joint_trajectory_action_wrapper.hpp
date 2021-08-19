#ifndef WRAPPER_H
#define WRAPPER_H

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <gsplines_follow_trajectory/FollowJointGSplineAction.h>
#include <ros/ros.h>

namespace gsplines_follow_trajectory {

class FollowJointTrajectoryActionWrapper {
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_prv_;
  actionlib::SimpleActionServer<FollowJointGSplineAction> action_server_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
      action_client_;

  FollowJointGSplineFeedback feedback_;
  FollowJointGSplineResult result_;
  std::string name_;
  std::string fjta_name_; // follow joint trajectory action name

  ros::Subscriber feedback_subscriber_;
  ros::Publisher feedback_repeater_;

public:
  FollowJointTrajectoryActionWrapper(const std::string &_name,
                                     const std::string &_fjta_name);
  virtual ~FollowJointTrajectoryActionWrapper() = default;
  void action_callback(const FollowJointGSplineGoalConstPtr &goal);

  void feedback_repeater_method(const FollowJointGSplineFeedbackConstPtr _msg);

  virtual void prehemption_action();
  virtual void done_action();
  virtual void active_action();
};

} // namespace gsplines_follow_trajectory

#endif /* WRAPPER_H */
