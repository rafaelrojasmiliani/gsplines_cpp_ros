#include <gsplines_ros/gsplines_ros.hpp>
#define EIGEN_TO_STD_VECTOR(_eigen_vector)                                     \
  (std::vector<double>(_eigen_vector.data(),                                   \
                       _eigen_vector.data() + _eigen_vector.size()))
namespace gsplines {

GSpline msg_to_gspline(const gsplines_msgs::GSpline &_msg) {

  std::unique_ptr<basis::Basis> basis = basis::string_to_basis(_msg.basis);

  std::pair<double, double> domain{_msg.domain_left_boundary,
                                   _msg.domain_right_boundary};

  std::size_t codom_dim = _msg.codom_dim;
  std::size_t number_of_intervals = _msg.number_of_intervals;

  Eigen::VectorXd coefficients = Eigen::Map<const Eigen::VectorXd>(
      _msg.coefficients.data(), _msg.coefficients.size());
  Eigen::VectorXd interval_lengths = Eigen::Map<const Eigen::VectorXd>(
      _msg.interval_lengths.data(), _msg.interval_lengths.size());

  return GSpline(domain, codom_dim, number_of_intervals, *basis, coefficients,
                 interval_lengths);
}

gsplines_msgs::GSpline gspline_to_msg(const GSpline &_gspline) {
  gsplines_msgs::GSpline result;

  result.basis = _gspline.get_basis().get_name();

  result.domain_left_boundary = _gspline.get_domain().first;
  result.domain_right_boundary = _gspline.get_domain().second;

  result.codom_dim = _gspline.get_codom_dim();

  result.number_of_intervals = _gspline.get_number_of_intervals();
  result.coefficients = EIGEN_TO_STD_VECTOR(_gspline.get_coefficients());

  result.interval_lengths =
      EIGEN_TO_STD_VECTOR(_gspline.get_interval_lengths());

  return std::move(result);
}

trajectory_msgs::JointTrajectory gspline_to_joint_trajectory_message(
    const GSpline &_gspline, const std::vector<std::string> &_joint_names,
    const ros::Duration &_rate) {

  trajectory_msgs::JointTrajectory result;

  double t0 = _gspline.get_domain().first;
  double t1 = _gspline.get_domain().second;

  std::size_t number_of_segments = _gspline.get_domain_length() / _rate.toSec();

  Eigen::VectorXd time_spam =
      Eigen::VectorXd::LinSpaced(number_of_segments + 1, t0, t1);

  functions::FunctionExpression gspline_diff_1 = _gspline.derivate();
  functions::FunctionExpression gspline_diff_2 = gspline_diff_1.derivate();

  Eigen::MatrixXd gspline_evaluated = _gspline(time_spam);
  Eigen::MatrixXd gspline_diff_1_evaluated = gspline_diff_1(time_spam);
  Eigen::MatrixXd gspline_diff_2_evaluated = gspline_diff_2(time_spam);

  for (std::size_t uici = 0; uici < gspline_evaluated.rows(); uici++) {

    trajectory_msgs::JointTrajectoryPoint trj_point;

    trj_point.positions = EIGEN_TO_STD_VECTOR(gspline_evaluated.row(uici));

    trj_point.velocities =
        EIGEN_TO_STD_VECTOR(gspline_diff_1_evaluated.row(uici));

    trj_point.accelerations =
        EIGEN_TO_STD_VECTOR(gspline_diff_2_evaluated.row(uici));

    trj_point.time_from_start = ros::Duration(std::fabs(time_spam(uici) - t0));

    result.points.push_back(std::move(trj_point));
  }

  result.joint_names = _joint_names;

  std_msgs::Header header;
  header.stamp = ros::Time::now();

  result.header = header;

  return result;
}

control_msgs::FollowJointTrajectoryGoal gspline_to_follow_joint_trajectory_goal(
    const GSpline &_gspline, const std::vector<std::string> &_joint_names,
    const ros::Duration &_rate) {

  control_msgs::FollowJointTrajectoryGoal result;

  result.trajectory =
      gspline_to_joint_trajectory_message(_gspline, _joint_names, _rate);

  return result;
}

} // namespace gsplines