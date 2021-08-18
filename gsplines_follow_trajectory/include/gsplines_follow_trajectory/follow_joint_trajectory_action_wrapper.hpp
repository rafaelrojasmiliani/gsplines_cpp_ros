#ifndef WRAPPER_H
#define WRAPPER_H

#include <actionlib/server/simple_action_server.h>
#include <gsplines_follow_trajectory_control/FollowJointGSplineAction.h>
#include <ros/ros.h>

namespace gsplines_follow_trajectory {

class FollowJointTrajectoryActionWrapper {
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_prv_;
  actionlib::SimpleActionServer<FollowJointGSplineAction> action_server_;

  FollowJointGSplineFeedback feedback_;
  FollowJointGSplineResult result_;
  std::string name_;

public:
  FollowJointTrajectoryActionWrapper(const std::string &_name);
  virtual ~FollowJointTrajectoryActionWrapper();
  void action_callback(const FollowJointGSplineGoalConstPtr &goal);
};

} // namespace gsplines_follow_trajectory_control

#endif /* WRAPPER_H */
