

#include <cstddef>
#include <eigen3/Eigen/Core>
#include <gsplines/Basis/BasisLagrange.hpp>
#include <gsplines/Basis/BasisLegendre.hpp>
#include <gsplines/Collocation/GaussLobattoPointsWeights.hpp>
#include <gsplines/Interpolator.hpp>
#include <gsplines/Tools.hpp>
#include <gsplines_ros/gsplines_ros.hpp>
#include <gtest/gtest.h>
#include <random>

using namespace gsplines;
using namespace gsplines::basis;
using namespace gsplines::collocation;
using namespace gsplines_ros;

std::random_device rd;
std::mt19937 mt(rd());
std::uniform_int_distribution<std::size_t> uint_dist(2, 9);
TEST(Messages_Conversions, Basis) {
  for (std::size_t dim = 3; dim < 17; dim++) {
    // test basis lagrange
    Eigen::VectorXd points = Eigen::VectorXd::Random(dim);
    std::shared_ptr<BasisLagrange> basis_lagrange = BasisLagrange::get(points);
    EXPECT_TRUE(*basis_lagrange ==
                *basis_msg_to_basis(basis_to_basis_msg(*basis_lagrange)));

    // test basis legendre
    std::shared_ptr<BasisLegendre> basis_legendre = BasisLegendre::get(dim);
    EXPECT_TRUE(*basis_legendre ==
                *basis_msg_to_basis(basis_to_basis_msg(*basis_legendre)));

    EXPECT_FALSE(*basis_legendre == *basis_lagrange);
    EXPECT_TRUE(*basis_legendre != *basis_lagrange);
  }
}

TEST(Messages_Conversions, GSpines) {
  for (std::size_t dim = 4; dim < 17; dim += 2) {
    std::size_t codom_dim = uint_dist(mt);
    std::size_t n_intervals = uint_dist(mt);
    Eigen::VectorXd tau(Eigen::VectorXd::Random(n_intervals).array() + 1.5);
    Eigen::MatrixXd waypoints(
        Eigen::MatrixXd::Random(n_intervals + 1, codom_dim));
    // test Lagrange
    Eigen::VectorXd glp = legendre_gauss_lobatto_points(dim);
    std::shared_ptr<BasisLagrange> basis_lagrange = BasisLagrange::get(glp);

    GSpline curve_1 = interpolate(tau, waypoints, *basis_lagrange);
    GSpline curve_2 = gspline_msg_to_gspline(gspline_to_msg(curve_1));
    EXPECT_TRUE(tools::approx_equal(curve_1, curve_2, 1.0e-8));

    // test Legengre
    std::shared_ptr<BasisLegendre> basis_legendre = BasisLegendre::get(dim);
    GSpline curve_3 = interpolate(tau, waypoints, *basis_legendre);
    GSpline curve_4 = gspline_msg_to_gspline(gspline_to_msg(curve_3));
    EXPECT_TRUE(tools::approx_equal(curve_3, curve_4, 1.0e-8));

    // test goint gsplines
    GSpline curve_5 = gspline_msg_to_gspline(
        gspline_to_joint_gspline_msg(curve_1,
                                     std::vector<std::string>(codom_dim, ""))
            .gspline);
    EXPECT_TRUE(tools::approx_equal(curve_1, curve_5, 1.0e-8));
  }
}

TEST(Messages_Conversions, Trajectory_Message) {
  for (std::size_t dim = 4; dim < 17; dim += 2) {
    std::size_t codom_dim = uint_dist(mt);
    std::size_t n_intervals = uint_dist(mt);
    Eigen::VectorXd tau(Eigen::VectorXd::Random(n_intervals).array() + 1.5);
    Eigen::MatrixXd waypoints(
        Eigen::MatrixXd::Random(n_intervals + 1, codom_dim));
    Eigen::MatrixXd waypoints_2(
        Eigen::MatrixXd::Random(n_intervals + 1, codom_dim));
    // test Lagrange
    Eigen::VectorXd glp = legendre_gauss_lobatto_points(dim);
    std::shared_ptr<BasisLagrange> basis_lagrange = BasisLagrange::get(glp);

    GSpline curve_1 = interpolate(tau, waypoints, *basis_lagrange);
    GSpline curve_2 = interpolate(tau, waypoints_2, *basis_lagrange);

    EXPECT_TRUE(curve_1 == function_to_joint_trajectory_msg(
                               curve_1, std::vector<std::string>(codom_dim, ""),
                               ros::Duration(0.01)));

    EXPECT_TRUE(curve_2 != function_to_joint_trajectory_msg(
                               curve_1, std::vector<std::string>(codom_dim, ""),
                               ros::Duration(0.01)));
  }
}

int main(int argc, char **argv) {

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
