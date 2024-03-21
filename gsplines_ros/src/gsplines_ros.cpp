#include <gsplines/Basis/Basis.hpp>
#include <gsplines/Basis/BasisLegendre.hpp>
#include <gsplines/Functions/FunctionBase.hpp>
#include <gsplines/GSpline.hpp>
#include <gsplines/Interpolator.hpp>
#include <gsplines/Optimization/ipopt_solver.hpp>
#include <gsplines/Tools.hpp>
#include <gsplines_ros/gsplines_ros.hpp>

#include <gsplines_msgs/Basis.h>
#include <gsplines_msgs/FollowJointGSplineActionGoal.h>
#include <gsplines_msgs/FollowJointGSplineActionResult.h>
#include <gsplines_msgs/GSpline.h>
#include <gsplines_msgs/JointGSpline.h>

#include <ros/console.h>
#include <ros/duration.h>
#include <ros/time.h>

#include <std_msgs/Header.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryResult.h>

#define EIGEN_TO_STD_VECTOR(_eigen_vector)                                     \
  (std::vector<double>(_eigen_vector.data(),                                   \
                       _eigen_vector.data() + _eigen_vector.size())) // NOLINT
namespace gsplines_ros {

std::shared_ptr<gsplines::basis::Basis>
basis_msg_to_basis(const gsplines_msgs::Basis &_msg) {

  ROS_WARN_STREAM("gegin base " << _msg.name);
  return gsplines::basis::get_basis(_msg.name, _msg.dim, _msg.parameters);
}

gsplines_msgs::Basis basis_to_basis_msg(const gsplines::basis::Basis &_in) {
  gsplines_msgs::Basis result;
  result.name = _in.get_name();
  ROS_WARN_STREAM("converting base " << result.name << " into message");
  result.dim = static_cast<int>(_in.get_dim());
  result.parameters = EIGEN_TO_STD_VECTOR(_in.get_parameters()); // NOLINT
  return result;
}

gsplines::GSpline gspline_msg_to_gspline(const gsplines_msgs::GSpline &_msg) {

  const std::shared_ptr<gsplines::basis::Basis> basis =
      basis_msg_to_basis(_msg.basis);

  const std::pair<double, double> domain{_msg.domain_left_boundary,
                                         _msg.domain_right_boundary};

  const std::size_t codom_dim = _msg.codom_dim;
  const std::size_t number_of_intervals = _msg.number_of_intervals;

  const Eigen::VectorXd coefficients = Eigen::Map<const Eigen::VectorXd>(
      _msg.coefficients.data(), static_cast<long>(_msg.coefficients.size()));
  const Eigen::VectorXd interval_lengths = Eigen::Map<const Eigen::VectorXd>(
      _msg.interval_lengths.data(),
      static_cast<long>(_msg.interval_lengths.size()));

  return {domain, codom_dim,    number_of_intervals,
          *basis, coefficients, interval_lengths};
}

gsplines_msgs::GSpline gspline_to_msg(const gsplines::GSpline &_gspline) {
  gsplines_msgs::GSpline result;

  result.basis = basis_to_basis_msg(_gspline.get_basis());

  result.domain_left_boundary = _gspline.get_domain().first;
  result.domain_right_boundary = _gspline.get_domain().second;

  result.codom_dim = static_cast<int>(_gspline.get_codom_dim());

  result.number_of_intervals =
      static_cast<int>(_gspline.get_number_of_intervals());
  result.coefficients =
      EIGEN_TO_STD_VECTOR(_gspline.get_coefficients()); // NOLINT

  result.interval_lengths =
      EIGEN_TO_STD_VECTOR(_gspline.get_interval_lengths()); // NOLINT

  return result;
}

gsplines_msgs::JointGSpline
gspline_to_joint_gspline_msg(const gsplines::GSpline &_gspline,
                             const std::vector<std::string> &_joint_names) {

  gsplines_msgs::JointGSpline result;
  result.gspline = gspline_to_msg(_gspline);
  result.name = _joint_names;
  return result;
}

trajectory_msgs::JointTrajectory
function_to_joint_trajectory_msg(const gsplines::functions::FunctionBase &_trj,
                                 const std::vector<std::string> &_joint_names,
                                 const ros::Duration &_step,
                                 std_msgs::Header _header) {

  trajectory_msgs::JointTrajectory result;

  const double t0 = _trj.get_domain().first;
  const double t1 = _trj.get_domain().second;

  const std::size_t number_of_segments =
      static_cast<long>(_trj.get_domain_length() / _step.toSec());

  Eigen::VectorXd time_spam = Eigen::VectorXd::LinSpaced(
      static_cast<long>(number_of_segments) + 1, t0, t1);

  const gsplines::functions::FunctionExpression gspline_diff_1 =
      _trj.derivate();
  const gsplines::functions::FunctionExpression gspline_diff_2 =
      _trj.derivate(2);

  const Eigen::MatrixXd gspline_evaluated = _trj(time_spam);
  const Eigen::MatrixXd gspline_diff_1_evaluated = gspline_diff_1(time_spam);
  const Eigen::MatrixXd gspline_diff_2_evaluated = gspline_diff_2(time_spam);

  for (long uici = 0; uici < gspline_evaluated.rows(); uici++) {

    trajectory_msgs::JointTrajectoryPoint trj_point;

    for (long uicj = 0; uicj < static_cast<long>(_trj.get_codom_dim());
         uicj++) {
      trj_point.positions.push_back(gspline_evaluated(uici, uicj));

      trj_point.velocities.push_back(gspline_diff_1_evaluated(uici, uicj));

      trj_point.accelerations.push_back(gspline_diff_2_evaluated(uici, uicj));
    }

    trj_point.time_from_start = ros::Duration(std::fabs(time_spam(uici) - t0));

    result.points.push_back(std::move(trj_point));
  }

  result.joint_names = _joint_names;

  result.header = std::move(_header);

  return result;
}

trajectory_msgs::JointTrajectory gspline_to_minimal_joint_trajectory_msg(
    const gsplines::GSpline &_trj, const std::vector<std::string> &_joint_names,
    std_msgs::Header _header) {

  trajectory_msgs::JointTrajectory result;

  const Eigen::MatrixXd wp = _trj.get_waypoints();
  const Eigen::VectorXd &taus = _trj.get_interval_lengths();

  for (long uici = 0; uici < wp.rows(); uici++) {

    trajectory_msgs::JointTrajectoryPoint trj_point;

    for (long uicj = 0; uicj < static_cast<long>(_trj.get_codom_dim());
         uicj++) {
      trj_point.positions.push_back(wp(uici, uicj));
    }

    if (uici == 0) {
      trj_point.time_from_start = ros::Duration(0.0);
    } else {
      trj_point.time_from_start = ros::Duration(taus.head(uici).array().sum());
    }
    result.points.push_back(std::move(trj_point));
  }

  result.joint_names = _joint_names;

  result.header = std::move(_header);

  return result;
}

trajectory_msgs::JointTrajectory joint_gspline_msg_to_joint_trajectory_msg(
    const gsplines_msgs::JointGSpline &_trj, const ros::Duration &_step) {

  const gsplines::GSpline trj = gspline_msg_to_gspline(_trj.gspline);
  return function_to_joint_trajectory_msg(trj, _trj.name, _step, _trj.header);
}

control_msgs::FollowJointTrajectoryGoal
function_to_follow_joint_trajectory_goal(
    const gsplines::functions::FunctionBase &_trj,
    const std::vector<std::string> &_joint_names, const ros::Duration &_step,
    std_msgs::Header _header) {

  control_msgs::FollowJointTrajectoryGoal result;

  result.trajectory = function_to_joint_trajectory_msg(
      _trj, _joint_names, _step, std::move(_header));

  return result;
}

control_msgs::FollowJointTrajectoryGoal
gspline_msg_to_follow_joint_trajectory_goal(
    const gsplines_msgs::JointGSpline &_trj,
    const std::vector<std::string> &_joint_names, const ros::Duration &_step,
    std_msgs::Header _header) {

  gsplines::GSpline trj = gspline_msg_to_gspline(_trj.gspline);

  return function_to_follow_joint_trajectory_goal(trj, _joint_names, _step,
                                                  _header);
}

control_msgs::FollowJointTrajectoryGoal
joint_gspline_msg_to_follow_joint_trajectory_goal(
    const gsplines_msgs::JointGSpline _trj, const ros::Duration &_step) {

  gsplines::GSpline trj = gspline_msg_to_gspline(_trj.gspline);
  return function_to_follow_joint_trajectory_goal(trj, _trj.name, _step,
                                                  _trj.header);
}

gsplines_msgs::FollowJointGSplineFeedback
follow_joint_trajectory_feedback_to_follow_joint_gspline_feedback(
    const control_msgs::FollowJointTrajectoryFeedback &_msg) {

  gsplines_msgs::FollowJointGSplineFeedback result;

  result.joint_names = _msg.joint_names;

  result.actual = _msg.actual;

  result.desired = _msg.desired;

  result.error = _msg.error;

  result.header = std_msgs::Header();
  result.header.stamp = ros::Time::now();

  return result;
}

control_msgs::FollowJointTrajectoryGoal
follow_joint_gspline_goal_to_follow_joint_trajectory_goal(
    const gsplines_msgs::FollowJointGSplineGoal &_msg,
    const ros::Duration &_control_step) {

  control_msgs::FollowJointTrajectoryGoal result;

  result.goal_time_tolerance = _msg.goal_time_tolerance;
  result.goal_tolerance = _msg.goal_tolerance;
  result.path_tolerance = _msg.path_tolerance;

  result.trajectory =
      joint_gspline_msg_to_joint_trajectory_msg(_msg.gspline, _control_step);

  return result;
}
control_msgs::FollowJointTrajectoryResult
follow_joint_gspline_result_to_follow_joint_trajectory_result(
    const gsplines_msgs::FollowJointGSplineResult &_msg) {

  control_msgs::FollowJointTrajectoryResult result;

  result.error_code = _msg.error_code;
  result.error_string = _msg.error_string;

  return result;
}
gsplines_msgs::FollowJointGSplineResult
follow_joint_trajectory_result_to_follow_joint_gspline_result(
    const control_msgs::FollowJointTrajectoryResult &_msg) {
  gsplines_msgs::FollowJointGSplineResult result;

  result.error_code = _msg.error_code;
  result.error_string = _msg.error_string;

  return result;
}

trajectory_msgs::JointTrajectory gspline_msg_to_joint_trajectory_msg(
    const gsplines_msgs::GSpline &_trj,
    const std::vector<std::string> &_joint_names, const ros::Duration &_step,
    std_msgs::Header _header) {

  gsplines::GSpline trj = gspline_msg_to_gspline(_trj);

  return function_to_joint_trajectory_msg(trj, _joint_names, _step,
                                          std::move(_header));
}
Eigen::MatrixXd get_waypoints(const trajectory_msgs::JointTrajectory &_msg) {
  std::size_t codom_dim = _msg.joint_names.size();
  std::size_t number_of_points = _msg.points.size();
  Eigen::MatrixXd result(number_of_points, codom_dim);

  std::size_t point_counter = 0;

  for (const trajectory_msgs::JointTrajectoryPoint &point : _msg.points) {
    result.row(point_counter) =
        Eigen::Map<const Eigen::Matrix<double, 1, Eigen::Dynamic>>(
            point.positions.data(), codom_dim);
    point_counter++;
  }

  return result;
}

Eigen::VectorXd
interval_length_vector(const trajectory_msgs::JointTrajectory &_msg) {
  std::size_t number_of_points = _msg.points.size();

  Eigen::VectorXd result(number_of_points - 1);

  for (long i = 0; i < static_cast<long>(number_of_points - 1); i++) {
    result(i) = _msg.points[i + 1].time_from_start.toSec() -
                _msg.points[i].time_from_start.toSec();
  }

  return result;
}

gsplines::GSpline
interpolate_joint_trajectory(const trajectory_msgs::JointTrajectory &_msg,
                             const Eigen::VectorXd &_tau,
                             const gsplines::basis::Basis &_basis) {

  const Eigen::MatrixXd waypoints = get_waypoints(_msg);

  return gsplines::interpolate(_tau, waypoints, _basis);
}

gsplines::GSpline
interpolate_joint_trajectory(const trajectory_msgs::JointTrajectory &_msg,
                             const gsplines::basis::Basis &_basis) {

  const Eigen::MatrixXd waypoints = get_waypoints(_msg);
  const Eigen::VectorXd tau = interval_length_vector(_msg);

  return gsplines::interpolate(tau, waypoints, _basis);
}

gsplines::GSpline
minimum_sobolev_semi_norm(const trajectory_msgs::JointTrajectory &_msg,
                          const gsplines::basis::Basis &_basis,
                          std::vector<std::pair<std::size_t, double>> _weights,
                          double _exec_time) {

  Eigen::MatrixXd waypoints = get_waypoints(_msg);
  return gsplines::optimization::optimal_sobolev_norm(waypoints, _basis,
                                                      _weights, _exec_time);
}

trajectory_msgs::JointTrajectory minimum_sobolev_semi_norm_joint_trajectory(
    Eigen::MatrixXd _waypoints, std::vector<std::string> _joint_names,
    const gsplines::basis::Basis &_basis,
    std::vector<std::pair<std::size_t, double>> _weights, double _exec_time,
    const ros::Duration &_step, std_msgs::Header _header) {

  gsplines::GSpline trj = gsplines::optimization::optimal_sobolev_norm(
      _waypoints, _basis, _weights, _exec_time);
  return function_to_joint_trajectory_msg(trj, _joint_names, _step, _header);
}

trajectory_msgs::JointTrajectory minimum_sobolev_semi_norm_joint_trajectory(
    const Eigen::MatrixXd _waypoints,
    const std::vector<std::string> _joint_names,
    const gsplines::basis::Basis &_basis,
    std::vector<std::pair<std::size_t, double>> _weights,
    const Eigen::VectorXd _velocity_bound,
    const Eigen::VectorXd _acceleration_bound, //
    const ros::Duration &_step,                //
    const std::optional<double> &_exec_time,   //
    std_msgs::Header _header) {

  std::vector<double> velocity_bound =
      EIGEN_TO_STD_VECTOR(_velocity_bound); // NOLINT
  std::vector<double> acceleration_bound =
      EIGEN_TO_STD_VECTOR(_acceleration_bound); // NOLINT

  return minimum_sobolev_semi_norm_joint_trajectory(
      _waypoints, _joint_names, _basis, _weights, velocity_bound,
      acceleration_bound, _step, _exec_time, _header);
}

trajectory_msgs::JointTrajectory minimum_sobolev_semi_norm_joint_trajectory(
    const Eigen::MatrixXd _waypoints,
    const std::vector<std::string> _joint_names,
    const gsplines::basis::Basis &_basis,
    std::vector<std::pair<std::size_t, double>> _weights,
    const std::vector<double> &_velocity_bound,
    const std::vector<double> &_acceleration_bound, //
    const ros::Duration &_step,
    const std::optional<double> &_exec_time, //
    std_msgs::Header _header) {

  trajectory_msgs::JointTrajectory result;

  gsplines::GSpline trj = gsplines::optimization::optimal_sobolev_norm(
      _waypoints, _basis, _weights,
      _exec_time.has_value() ? _exec_time.value() : _waypoints.rows() - 1);

  auto trj2 =
      trj.linear_scaling_new_execution_time_max_velocity_max_acceleration(
          _velocity_bound, _acceleration_bound, _step.toSec());

  auto gspline_ = gspline_to_msg(trj2);
  result = gspline_msg_to_joint_trajectory_msg(gspline_, _joint_names, _step,
                                               _header);
  return result;
}

trajectory_msgs::JointTrajectory
minimum_jerk_trajectory(const Eigen::MatrixXd _waypoints,
                        const std::vector<std::string> _joint_names,
                        const ros::Duration &_duration,
                        const ros::Duration &_step, std_msgs::Header _header) {

  return minimum_sobolev_semi_norm_joint_trajectory(
      _waypoints, _joint_names, gsplines::basis::BasisLegendre(6), {{3, 1.0}},
      _duration.toSec(), _step, std::move(_header));
}

trajectory_msgs::JointTrajectory
minimum_jerk_trajectory(const Eigen::MatrixXd _waypoints,
                        const std::vector<std::string> _joint_names,
                        const Eigen::VectorXd _velocity_bound,
                        const Eigen::VectorXd _acceleration_bound,
                        const ros::Duration &_step, std_msgs::Header _header) {

  return minimum_sobolev_semi_norm_joint_trajectory(
      _waypoints, _joint_names, gsplines::basis::BasisLegendre(6), {{3, 1.0}},
      _velocity_bound, _acceleration_bound, _step, std::nullopt, _header);
}

trajectory_msgs::JointTrajectory
minimum_jerk_trajectory(const Eigen::MatrixXd _waypoints,
                        const std::vector<std::string> _joint_names,
                        const std::vector<double> _velocity_bound,
                        const std::vector<double> _acceleration_bound,
                        const ros::Duration &_step, std_msgs::Header _header) {

  Eigen::VectorXd velocity_bound = Eigen::Map<const Eigen::VectorXd>(
      _velocity_bound.data(), _velocity_bound.size());
  Eigen::VectorXd acceleration_bound = Eigen::Map<const Eigen::VectorXd>(
      _acceleration_bound.data(), _acceleration_bound.size());

  return minimum_jerk_trajectory(_waypoints, _joint_names, velocity_bound,
                                 acceleration_bound, _step, _header);
}

} // namespace gsplines_ros

bool operator==(const trajectory_msgs::JointTrajectory &_msg,
                const gsplines::functions::FunctionBase &_fun) {
  std::vector<double> time_span_vector;
  std::transform(_msg.points.cbegin(), _msg.points.cend(),
                 std::back_inserter(time_span_vector),
                 [](const trajectory_msgs::JointTrajectoryPoint &_point) {
                   return _point.time_from_start.toSec();
                 });
  if (_msg.points.empty() or
      _msg.points.begin()->positions.size() != _fun.get_codom_dim()) {
    return false;
  }
  Eigen::Map<Eigen::VectorXd> time_spam(time_span_vector.data(),
                                        time_span_vector.size());

  time_spam.array() += _fun.get_domain().first;

  Eigen::MatrixXd gspline_evaluated = _fun(time_spam);

  for (long uici = 0; uici < gspline_evaluated.rows(); uici++) {

    const trajectory_msgs::JointTrajectoryPoint &trj_point = _msg.points[uici];

    if (trj_point.positions.empty() or
        trj_point.positions.size() != _fun.get_codom_dim()) {
      return false;
    }

    if (not gsplines::tools::approx_equal(
            gspline_evaluated.row(uici),
            Eigen::Map<const Eigen::Matrix<double, 1, Eigen::Dynamic>>(
                trj_point.positions.data(), trj_point.positions.size()),
            1.0e-7)) {
      return false;
    }
  }
  return true;
}
bool operator!=(const trajectory_msgs::JointTrajectory &_msg,
                const gsplines::functions::FunctionBase &_fun) {
  return not(_msg == _fun);
}
bool operator==(const gsplines::functions::FunctionBase &_fun,
                const trajectory_msgs::JointTrajectory &_msg) {
  return _msg == _fun;
}
bool operator!=(const gsplines::functions::FunctionBase &_fun,
                const trajectory_msgs::JointTrajectory &_msg) {
  return _msg != _fun;
}
