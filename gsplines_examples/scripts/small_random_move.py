#! /usr/bin/env python
"""
    This Example teach to implement a Minimum jerk trajectory with 
    a FollowJointTrajectoryAction.
    It does the following procedure
    0. Connects with the action server
    1. Get the actual position of the robot
    2. Compute a small minimum jerk random motion that starts and arrives to
       the actual position of the robot. This returns a gspline
    3. Scale the time execution of the gspline.
    4. Convert the gspline into a follow joint trajectory goal
    5. Sends the joint trajectory goal to the robot.

    This example has three main components
    1. Defaul instantiaion of the ROS enviromment
        - Parameters 
        - Connection to FollowJointTrajectoryAction
    2. Feedback from the robot to get the actual position and compute a small
        random motion which starts and ends at this position
    3. Gspline computation and conversion
"""
import numpy as np
import rospy
from control_msgs.msg import FollowJointTrajectoryAction
from gsplines import minimum_jerk_path
from gsplines_ros import gspline_to_follow_joint_trajectory_goal
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import actionlib
from actionlib_msgs.msg import GoalStatus


def get_actual_joint_position(_joint_names: list):
    """
    Get the current state of the rboto.  This functoion listens for the topic
    joint_states in the current namespace.  Takes only one message a extract
    the position of the desired joints joint names

    :param _joint_names list: Desired joints
    :raises ValueError: if it cannot get the desired joints
    """

    try:
        joint_state_msg = rospy.wait_for_message(
            'joint_states', JointState, timeout=rospy.Duration.from_sec(5.0))
    except rospy.ROSInterruptException:
        raise ValueError('cannot get the sate')

    list_of_pairs = dict(
        list(zip(joint_state_msg.name, joint_state_msg.position)))

    return np.array([list_of_pairs[name] for name in _joint_names])


def get_header_with_delay(_delay_seconds: float):
    """
    #returns a Header with a timestam equato to now + _delay_seconds

    :param _delay_seconds float: number of seconds of the delay from now
    """
    if _delay_seconds < 1.0e-10:
        return Header()
    result = Header()
    result.stamp = rospy.Time.now() + rospy.Duration.from_sec(_delay_seconds)

    return result


def main():
    """
        Generated a random mimum jerk trajectory and sends it
        to a FollowJointTrajectoryAction server.
    """
    rospy.init_node('random_minimum_jerk_motions')

    # -------------------------------
    # 1. Initialize parameters
    # ------------------------------
    params = {}
    # resolution of the control (time steps in FollowJointTrajectory)
    params['control_step'] = 0.01
    params['number_of_waypoints'] = 3
    # name of the control that implement follow_joint_trajectory
    # (its namespace)
    params['control_name'] = ''
    params['waypoint_radius'] = 0.05  # magnitute of the random motion
    params['execution_time'] = 1.0  # time to execute the motion
    # delay of the network. Motion will start from after such a time has passed
    params['network_delay_milliseconds'] = 100

    for key, value in params.items():
        # -------------------------------
        # 1.1 Get the names of the joints that the control controls
        # ------------------------------
        params[key] = rospy.get_param('~'+key, value)

    try:
        joint_names = rospy.get_param(params['control_name']+'/joints')
    except KeyError:
        rospy.logerr_once('no joint names defined on control. cannot proceed')
        return

    # -------------------------------
    # 2. Connect to the action server
    # ------------------------------
    motion_action = actionlib.SimpleActionClient(
        params['control_name']+'/follow_joint_trajectory',
        FollowJointTrajectoryAction)

    rospy.loginfo("Waiting for action " +
                  params['control_name']+'/follow_joint_trajectory')
    server_is_up = motion_action.wait_for_server()
    if not server_is_up:
        rospy.logerr_once(
            'No action follow_joint_trajectory in namespace {:d}'.format(
                params['control_name']))
        return
    rospy.loginfo('Connected to the action server ' +
                  params['control_name']+'/follow_joint_trajectory')

    # -------------------------------
    # 3. Get initial position of the robot
    # ------------------------------
    initial_position = get_actual_joint_position(joint_names)
    codom_dim = len(initial_position)

    # ---------------------------------------------------------------
    # 4. Compute a random set of waypoints in the neighborhood of the
    #    initial position of the robot. The final and inital waypoints
    #    are the current position of the robot. The motion is cyclic
    # ---------------------------------------------------------------
    waypoints = np.random.rand(
        params['number_of_waypoints'], codom_dim)

    waypoints = (2.0*waypoints - 1.0) * params['waypoint_radius']

    waypoints += initial_position

    waypoints[0, :] = initial_position
    waypoints[-1, :] = initial_position

    # ---------------------------------------------------------------
    # 5. Compute minimum jerk motion and linearly scale such a motion
    #    up to the desired execution time.
    # ---------------------------------------------------------------
    trj = minimum_jerk_path(waypoints)
    trj = trj.linear_scaling_new_execution_time(params['execution_time'])

    # ---------------------------------------------------------------
    # 6. Generate the goal
    # ---------------------------------------------------------------
    # ---------------------------------------------------------------
    # 6.1. Assign to the trajectory an execution time in the future
    #    taking into account the network delay.
    # ---------------------------------------------------------------
    header = get_header_with_delay(
        params['network_delay_milliseconds']*1.0e-3)

    # 6.2 Generate the goal
    goal = gspline_to_follow_joint_trajectory_goal(
        trj, joint_names, rospy.Duration(0.01), header)

    # 6.3 send the goal and wait
    goal_state = motion_action.send_goal_and_wait(goal)

    if goal_state != GoalStatus.SUCCEEDED:
        rospy.logerr('Action did not succeeded :' +
                     motion_action.get_goal_status_text())
        if not params['persistent']:
            return

    initial_position = get_actual_joint_position(joint_names)


if __name__ == '__main__':
    main()
