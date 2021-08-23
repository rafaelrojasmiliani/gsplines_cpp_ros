#include <gsplines_ros/gsplines_ros.hpp>
#define EIGEN_TO_STD_VECTOR(_eigen_vector)                                     \
  (std::vector<double>(_eigen_vector.data(),                                   \
                       _eigen_vector.data() + _eigen_vector.size()))
namespace gsplines_ros {

gsplines::GSpline msg_to_gspline(const gsplines_msgs::GSpline &_msg) {

  std::unique_ptr<gsplines::basis::Basis> basis =
      gsplines::basis::string_to_basis(_msg.basis);

  std::pair<double, double> domain{_msg.domain_left_boundary,
                                   _msg.domain_right_boundary};

  std::size_t codom_dim = _msg.codom_dim;
  std::size_t number_of_intervals = _msg.number_of_intervals;

  Eigen::VectorXd coefficients = Eigen::Map<const Eigen::VectorXd>(
      _msg.coefficients.data(), _msg.coefficients.size());
  Eigen::VectorXd interval_lengths = Eigen::Map<const Eigen::VectorXd>(
      _msg.interval_lengths.data(), _msg.interval_lengths.size());

  return gsplines::GSpline(domain, codom_dim, number_of_intervals, *basis,
                           coefficients, interval_lengths);
}

gsplines_msgs::GSpline gspline_to_msg(const gsplines::GSpline &_gspline) {
  gsplines_msgs::GSpline result;

  result.basis = _gspline.get_basis_name();

  result.domain_left_boundary = _gspline.get_domain().first;
  result.domain_right_boundary = _gspline.get_domain().second;

  result.codom_dim = _gspline.get_codom_dim();

  result.number_of_intervals = _gspline.get_number_of_intervals();
  result.coefficients = EIGEN_TO_STD_VECTOR(_gspline.get_coefficients());

  result.interval_lengths =
      EIGEN_TO_STD_VECTOR(_gspline.get_interval_lengths());

  return std::move(result);
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
gspline_to_joint_trajectory_msgs(const gsplines::GSpline &_gspline,
                                 const std::vector<std::string> &_joint_names,
                                 const ros::Duration &_step) {

  trajectory_msgs::JointTrajectory result;

  double t0 = _gspline.get_domain().first;
  double t1 = _gspline.get_domain().second;

  std::size_t number_of_segments = _gspline.get_domain_length() / _step.toSec();

  Eigen::VectorXd time_spam =
      Eigen::VectorXd::LinSpaced(number_of_segments + 1, t0, t1);

  gsplines::functions::FunctionExpression gspline_diff_1 = _gspline.derivate();
  gsplines::functions::FunctionExpression gspline_diff_2 = _gspline.derivate(2);

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

trajectory_msgs::JointTrajectory gspline_msg_to_joint_trajectory_msgs(
    const gsplines_msgs::GSpline _trj,
    const std::vector<std::string> _joint_names, const ros::Duration &_step) {

  gsplines::GSpline trj = msg_to_gspline(_trj);

  return gspline_to_joint_trajectory_msgs(trj, _joint_names, _step);
}

trajectory_msgs::JointTrajectory joint_gspline_msg_to_joint_trajectory_msgs(
    const gsplines_msgs::JointGSpline &_trj, const ros::Duration &_step) {

  gsplines::GSpline trj = msg_to_gspline(_trj.gspline);
  return gspline_to_joint_trajectory_msgs(trj, _trj.name, _step);
}

control_msgs::FollowJointTrajectoryGoal gspline_to_follow_joint_trajectory_goal(
    const gsplines::GSpline &_gspline,
    const std::vector<std::string> &_joint_names, const ros::Duration &_step) {

  control_msgs::FollowJointTrajectoryGoal result;

  result.trajectory =
      gspline_to_joint_trajectory_msgs(_gspline, _joint_names, _step);

  return result;
}

control_msgs::FollowJointTrajectoryGoal
gspline_msgs_to_follow_joint_trajectory_goal(
    const gsplines_msgs::JointGSpline &_trj,
    const std::vector<std::string> &_joint_names, const ros::Duration &_step) {

  gsplines::GSpline trj = msg_to_gspline(_trj.gspline);

  return gspline_to_follow_joint_trajectory_goal(trj, _joint_names, _step);
}

control_msgs::FollowJointTrajectoryGoal
joint_gspline_msgs_to_follow_joint_trajectory_goal(
    const gsplines_msgs::JointGSpline _trj, const ros::Duration &_step) {

  gsplines::GSpline trj = msg_to_gspline(_trj.gspline);
  return gspline_to_follow_joint_trajectory_goal(trj, _trj.name, _step);
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

  return std::move(result);
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
      joint_gspline_msg_to_joint_trajectory_msgs(_msg.gspline, _control_step);

  return std::move(result);
}
control_msgs::FollowJointTrajectoryResult
follow_joint_gspline_result_to_follow_joint_trajectory_result(
    const gsplines_msgs::FollowJointGSplineResult &_msg,
    const ros::Duration &_control_step) {

  control_msgs::FollowJointTrajectoryResult result;

  result.error_code = _msg.error_code;
  result.error_string = _msg.error_string;

  return std::move(result);
}
gsplines_msgs::FollowJointGSplineResult
follow_joint_trajectory_result_to_follow_joint_gspline_result(
    const control_msgs::FollowJointTrajectoryResult &_msg) {
  gsplines_msgs::FollowJointGSplineResult result;

  result.error_code = _msg.error_code;
  result.error_string = _msg.error_string;

  return std::move(result);
}
} // namespace gsplines_ros
