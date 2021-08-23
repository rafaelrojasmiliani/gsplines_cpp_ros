"""
Message conversions for gsplines
"""

import rospy

import gsplines
from gsplines import PyInterpolator as Interpolator
from gsplines import string_to_basis
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import JointTolerance
from control_msgs.msg import FollowJointTrajectoryGoal

from std_msgs.msg import Header

from gsplines_msgs.msg import GSpline, JointGSpline
import numpy as np


def gspline_msg_to_gspline(_msg: GSpline):

    basis = string_to_basis(_msg.basis)

    domain = (_msg.domain_left_boundary, _msg.domain_right_boundary)

    codom_dim = _msg.codom_dim

    number_of_intervals = _msg.number_of_intervals

    coefficients = _msg.coefficients

    interval_lengths = _msg.interval_lengths

    return gsplines.GSpline(domain, codom_dim, number_of_intervals,
                            basis, coefficients, interval_lengths)


def joint_gspline_msg_to_gspline(_msg: JointGSpline):

    basis = string_to_basis(_msg.gspline.basis)

    domain = (_msg.gspline.domain_left_boundary,
              _msg.gspline.domain_right_boundary)

    codom_dim = _msg.gspline.codom_dim

    number_of_intervals = _msg.gspline.number_of_intervals

    coefficients = _msg.gspline.coefficients

    interval_lengths = _msg.gspline.interval_lengths

    return gsplines.GSpline(domain, codom_dim, number_of_intervals,
                            basis, coefficients, interval_lengths)


def gspline_to_msg(_gspline: gsplines.GSpline):
    result = GSpline()

    result.basis = _gspline.get_basis_name()

    result.domain_left_boundary = _gspline.get_domain()[0]
    result.domain_right_boundary = _gspline.get_domain()[1]

    result.codom_dim = _gspline.get_codom_dim()

    result.number_of_intervals = _gspline.get_number_of_intervals()

    result.coefficients = _gspline.get_coefficients().copy()
    result.interval_lengths = _gspline.get_interval_lengths().copy()

    return result


def gspline_to_joint_gspline_msg(_gspline, _joint_names):
    """
    Transforms a Gsplines into a JointGSpline message
    """

    result = JointGSpline()
    result.gspline = gspline_to_msg(_gspline)
    result.name = _joint_names
    return result


def gspline_to_joint_trajectory_msg(_gspline, _joint_names,
                                    _step: rospy.Duration):
    """
    converts a gspline into a JointTrajectory message

    :param _gspline gsplines.GSpline: gspline
    :param _joint_names array: array of joint names
    :param _step rospy.Duration: time distance between samples
    """

    result = JointTrajectory()

    time_0, time_1 = _gspline.get_domain()

    time_spam = np.arange(time_0, time_1+_step.to_sec(), _step.to_sec())

    gspline_diff_1 = _gspline.deriv()
    gspline_diff_2 = _gspline.deriv(2)

    gspline_evaluated = _gspline(time_spam)
    gspline_diff_1_evaluated = gspline_diff_1(time_spam)
    gspline_diff_2_evaluated = gspline_diff_2(time_spam)

    for i, time_i in enumerate(time_spam):
        trjpoint = JointTrajectoryPoint()
        trjpoint.positions = list(gspline_evaluated[i, :])
        trjpoint.velocities = list(gspline_diff_1_evaluated[i, :])
        trjpoint.accelerations = list(gspline_diff_2_evaluated[i, :])
        trjpoint.time_from_start = rospy.Duration.from_sec(time_i)
        result.points.append(trjpoint)

    header = Header()
    header.stamp = rospy.Time()
    result.header = header
    result.joint_names = _joint_names
    return result


def gspline_msg_to_joint_trajectory_msg(_gspline_msg, _joint_names, _step):
    gspline = gspline_msg_to_gspline(_gspline_msg)
    return gspline_to_joint_trajectory_msg(gspline, _joint_names, _step)


def joint_gspline_msg_to_joint_trajectory_msg(_joint_gspline_msg:
                                              JointGSpline, _step):

    return gspline_msg_to_joint_trajectory_msg(_joint_gspline_msg.gspline,
                                               _joint_gspline_msg.name, _step)


def gspline_to_follow_joint_trajectory_goal(_gspline, _joint_names,
                                            _step: rospy.Duration):

    result = FollowJointTrajectoryGoal()

    result.trajectory = gspline_to_joint_trajectory_msg(
        _gspline, _joint_names, _step)

    return result


def gspline_msg_to_follow_joint_trajectory_goal(_gspline_msg,
                                                _joint_names, _step):

    trj = gspline_msg_to_gspline(_gspline_msg)
    return gspline_msg_to_follow_joint_trajectory_goal(trj,
                                                       _joint_names, _step)


def joint_gspline_msgs_to_follow_joint_trajectory_goal(_joint_gspline_msg,
                                                       _step):
    trj = gspline_msg_to_gspline(_joint_gspline_msg.gspline)
    return gspline_msg_to_follow_joint_trajectory_goal(trj,
                                                       _joint_gspline_msg.name,
                                                       _step)
