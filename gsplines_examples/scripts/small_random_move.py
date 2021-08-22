import rospy
import actionlib
from gsplines import minimum_jerk_path
from gsplines_ros import gspline_msgs_to_follow_joint_trajectory_goal
from sensor_msgs.msg import JointState
import numpy as np
from gspline_msgs.msg import FollowJointGSplineAction


def main():
    rospy.init_node('myNodeName', anonymous=True)
    joint_state = rospy.wait_for_message("joint_state", JointState)

    number_of_points = 5

    execution_time = 3.0

    pose = joint_state.position
    joint_names = joint_state.name

    random_disturbance = np.random.rand(number_of_points, len(pose))*0.01

    waypoints = np.einsum('ij,j->ij', random_disturbance, pose)
    waypoints[0, :] = pose
    waypoints[-1, :] = pose

    trajectory = minimum_jerk_path(
        waypoints).linear_scaling_new_execution_time(execution_time)

    goal = gspline_msgs_to_follow_joint_trajectory_goal(
        trajectory, joint_names, 125)

    self.motion_action_ = actionlib.SimpleActionClient(
        ctr_ns, FollowJointTrajectoryAction)


if __name__ == "__main__":
    main()
