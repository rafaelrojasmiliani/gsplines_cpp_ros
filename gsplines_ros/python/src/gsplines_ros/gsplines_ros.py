"""
Message conversions for gsplines
"""

import rospy

import gsplines
from gsplines.basis import get_basis
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import FollowJointTrajectoryGoal

from std_msgs.msg import Header

from gsplines_msgs.msg import GSpline, JointGSpline, Basis
import numpy as np
from typing import List


def basis_msg_to_basis(_msg: Basis) -> gsplines.basis.Basis:
    a = gsplines.basis.get_basis(_msg.name, _msg.dim, _msg.parameters)
    return a


def basis_to_basis_msg(_basis: gsplines.basis.Basis) -> Basis:
    result = Basis()
    result.name = _basis.get_name()
    result.dim = _basis.get_dim()
    result.parameters = _basis.get_parameters()
    return result


def gspline_to_msg(_gspline: gsplines.GSpline):
    """ GSpline message to GSpline message """
    result = GSpline()

    result.basis = basis_to_basis_msg(_gspline.get_basis())

    result.domain_left_boundary = _gspline.get_domain()[0]
    result.domain_right_boundary = _gspline.get_domain()[1]

    result.codom_dim = _gspline.get_codom_dim()

    result.number_of_intervals = _gspline.get_number_of_intervals()

    result.coefficients = _gspline.get_coefficients().copy()
    result.interval_lengths = _gspline.get_interval_lengths().copy()

    return result


def gspline_msg_to_gspline(_msg: GSpline) -> gsplines.GSpline:
    """ GSpline message to GSpline """

    basis = basis_msg_to_basis(_msg.basis)
    domain = (_msg.domain_left_boundary, _msg.domain_right_boundary)

    codom_dim = _msg.codom_dim

    number_of_intervals = _msg.number_of_intervals

    coefficients = _msg.coefficients

    interval_lengths = _msg.interval_lengths

    return gsplines.GSpline(domain, codom_dim, number_of_intervals,
                            basis, coefficients, interval_lengths, "gspline")


def joint_gspline_msg_to_gspline(_msg: JointGSpline) -> gsplines.GSpline:
    """ Joint GSpline message to GSpline """

    basis = get_basis(_msg.basis.name, _msg.basis.dim, _msg.basis.parameters)

    domain = (_msg.gspline.domain_left_boundary,
              _msg.gspline.domain_right_boundary)

    codom_dim = _msg.gspline.codom_dim

    number_of_intervals = _msg.gspline.number_of_intervals

    coefficients = _msg.gspline.coefficients

    interval_lengths = _msg.gspline.interval_lengths

    return gsplines.GSpline(domain, codom_dim, number_of_intervals,
                            basis, coefficients, interval_lengths, "gspline")


def gspline_to_joint_gspline_msg(_gspline: gsplines.GSpline,
                                 _joint_names: list,
                                 _header: Header = Header()):
    """
    Transforms a Gsplines into a JointGSpline message
    """

    result = JointGSpline()
    result.gspline = gspline_to_msg(_gspline)
    result.name = _joint_names
    result.header = _header
    return result


def gspline_to_joint_trajectory_msg(_gspline, _joint_names,
                                    _step: rospy.Duration,
                                    _header: Header = Header()):
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

    result.header = _header
    result.joint_names = _joint_names
    return result


def gspline_msg_to_joint_trajectory_msg(_gspline_msg: GSpline,
                                        _joint_names: List[str],
                                        _step: rospy.Duration,
                                        _header: Header = Header()):
    """

    :param _gspline_msg gsplines_msgs.msg.GSpline: gspline
    :param _joint_names list of joint ames: joint names
    :param _step control step: control steo
    """
    gspline = gspline_msg_to_gspline(_gspline_msg)
    return gspline_to_joint_trajectory_msg(gspline,
                                           _joint_names,
                                           _step, _header)


def joint_gspline_msg_to_joint_trajectory_msg(_joint_gspline_msg:
                                              JointGSpline,
                                              _step: rospy.Duration):
    """
    converst a JointGSpline message into a JointTrajectory nessage

    :param _joint_gspline_msg JointGSpline: [TODO:description]
    :param _step rospy.Duration: [TODO:description]
    :param _header Header: [TODO:description]
    """

    return gspline_msg_to_joint_trajectory_msg(_joint_gspline_msg.gspline,
                                               _joint_gspline_msg.name,
                                               _step,
                                               _joint_gspline_msg.header)


def gspline_to_follow_joint_trajectory_goal(_gspline: gsplines.GSpline,
                                            _joint_names: List[str],
                                            _step: rospy.Duration,
                                            _header: Header = Header()):

    result = FollowJointTrajectoryGoal()

    result.trajectory = gspline_to_joint_trajectory_msg(
        _gspline, _joint_names, _step, _header)

    return result


def gspline_msg_to_follow_joint_trajectory_goal(_gspline_msg: GSpline,
                                                _joint_names: List[str],
                                                _step: rospy.Duration,
                                                _header: Header = Header()):

    trj = gspline_msg_to_gspline(_gspline_msg)
    return gspline_msg_to_follow_joint_trajectory_goal(trj,
                                                       _joint_names, _step,
                                                       _header)


def joint_gspline_msgs_to_follow_joint_trajectory_goal(_joint_gspline_msg:
                                                       JointGSpline,
                                                       _step: rospy.Duration,
                                                       _header:
                                                       Header = Header()):

    trj = gspline_msg_to_gspline(_joint_gspline_msg.gspline)
    return gspline_msg_to_follow_joint_trajectory_goal(trj,
                                                       _joint_gspline_msg.name,
                                                       _step, _header)
