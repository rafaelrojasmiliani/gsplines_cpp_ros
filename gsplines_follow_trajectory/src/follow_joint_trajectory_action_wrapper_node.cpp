#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <gsplines_follow_trajectory/follow_joint_trajectory_action_wrapper.hpp>

int main(int argc, char **argv) {

  ros::init(argc, argv, "wrapper");
  ros::NodeHandle node("~");
  XmlRpc::XmlRpcValue xmlval;
  double control_step = 0.01;
  std::string action_name = "follow_joint_gspline";
  std::string target_action_name = "pos_joint_traj_controller";

  if (node.getParam("control_step", xmlval) and
      xmlval.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    control_step = xmlval;
  } else {
    ROS_INFO("control step %+14.7e", control_step);
  }
  if (node.getParam("action_name", xmlval) and
      xmlval.getType() == XmlRpc::XmlRpcValue::TypeString) {
    action_name = static_cast<std::string>(xmlval);
  } else {
    ROS_INFO("control step %s", action_name.c_str());
  }
  if (node.getParam("target_action_ns", xmlval) and
      xmlval.getType() == XmlRpc::XmlRpcValue::TypeString) {
    target_action_name = static_cast<std::string>(xmlval);
  } else {
    ROS_INFO("control step %s", target_action_name.c_str());
  }

  gsplines_follow_trajectory::FollowJointTrajectoryActionWrapper wrapper(
      action_name, target_action_name, control_step);

  ros::spin();

  return 0;
}
