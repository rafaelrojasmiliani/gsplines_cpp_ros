
#include <gsplines/Basis/BasisLagrange.hpp>
#include <gsplines/Basis/BasisLegendre.hpp>
#include <gsplines/Collocation/GaussLobattoPointsWeights.hpp>
#include <gsplines/Interpolator.hpp>
#include <gsplines_msgs/Basis.h>
#include <gsplines_msgs/FollowJointGSplineFeedback.h>
#include <gsplines_msgs/FollowJointGSplineGoal.h>
#include <gsplines_msgs/FollowJointGSplineResult.h>
#include <gsplines_msgs/GSpline.h>
#include <gsplines_msgs/JointGSpline.h>
#include <gsplines_ros/gsplines_ros.hpp>
#include <gtest/gtest.h>
#include <random>
#include <ros/ros.h>

// Import namespaces
using namespace gsplines::collocation;
using namespace gsplines;
using namespace gsplines::basis;
using namespace gsplines_ros;

// Instantiate random number generations (c++17)
std::random_device rd;
std::mt19937 mt(rd());
std::uniform_int_distribution<std::size_t> uint_dist(2, 9);

std::string get_random_string(const int len) {
  static const char alphanum[] = "0123456789"
                                 "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                                 "abcdefghijklmnopqrstuvwxyz";
  std::string tmp_s;
  tmp_s.reserve(len);

  for (int i = 0; i < len; ++i) {
    tmp_s += alphanum[rand() % (sizeof(alphanum) - 1)];
  }

  return tmp_s;
}

// Test that the basis of lagrange are sent correctly
TEST(GSplines_Messages, BasisLagrange) {
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<gsplines_msgs::Basis>("basis", 1);
  ros::spinOnce();

  for (std::size_t dim = 3; dim < 10; dim++) {
    Eigen::VectorXd points = Eigen::VectorXd::Random(dim);
    std::shared_ptr<Basis> basis_lagrange = BasisLagrange::get(points);
    std::shared_ptr<Basis> basis_lagrange_2 = nullptr;

    ros::Subscriber sub = nh.subscribe<gsplines_msgs::Basis>(
        "basis", 0,
        [&basis_lagrange_2](const gsplines_msgs::BasisConstPtr &_msg) -> void {
          basis_lagrange_2 = basis_msg_to_basis(*_msg);
        });

    int counter = 0;
    while (ros::ok() and counter < 30) {
      pub.publish(basis_to_basis_msg(*basis_lagrange));
      ros::spinOnce();
      ros::WallDuration(0.01).sleep();

      if (basis_lagrange_2) {
        EXPECT_TRUE(*basis_lagrange == *basis_lagrange_2);
      }
      counter++;
    }
    EXPECT_TRUE(basis_lagrange_2);
  }
}

// Test that the basis of legendre are sent correctly
TEST(GSplines_Messages, BasisLegendre) {
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<gsplines_msgs::Basis>("basis", 1);
  ros::spinOnce();

  for (std::size_t dim = 3; dim < 10; dim++) {
    std::shared_ptr<Basis> basis_legendre = BasisLegendre::get(dim);
    std::shared_ptr<Basis> basis_legendre_2 = nullptr;

    ros::Subscriber sub = nh.subscribe<gsplines_msgs::Basis>(
        "basis", 0,
        [&basis_legendre_2](const gsplines_msgs::BasisConstPtr &_msg) -> void {
          basis_legendre_2 = basis_msg_to_basis(*_msg);
        });

    int counter = 0;
    while (ros::ok() and counter < 30) {
      pub.publish(basis_to_basis_msg(*basis_legendre));
      ros::spinOnce();
      ros::WallDuration(0.01).sleep();

      if (basis_legendre_2) {
        EXPECT_TRUE(*basis_legendre == *basis_legendre_2);
      }
      counter++;
    }
    EXPECT_TRUE(basis_legendre_2);
  }
}

// Test that GSplines with lagrange basis are sent correctly
TEST(TestSuite, testCase2) {

  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<gsplines_msgs::JointGSpline>("gspline", 1);
  std::shared_ptr<GSpline> gspline_received = nullptr;
  ros::Subscriber sub = nh.subscribe<gsplines_msgs::GSpline>(
      "gspline", 0,
      [&gspline_received](const gsplines_msgs::GSplineConstPtr &_msg) -> void {
        gspline_received =
            std::make_shared<GSpline>(gspline_msg_to_gspline(*_msg));
      });
  for (std::size_t dim = 4; dim < 17; dim += 2) {
    std::size_t codom_dim = uint_dist(mt);
    std::vector<std::string> joint_names;
    for (std::size_t i = 0; i < codom_dim; i++) {
      joint_names.push_back(get_random_string(5));
    }
    std::size_t n_intervals = uint_dist(mt);
    Eigen::VectorXd tau(Eigen::VectorXd::Random(n_intervals).array() + 1.5);
    Eigen::MatrixXd waypoints(
        Eigen::MatrixXd::Random(n_intervals + 1, codom_dim));
    // test Lagrange
    Eigen::VectorXd glp = legendre_gauss_lobatto_points(dim);
    std::shared_ptr<BasisLagrange> basis_lagrange = BasisLagrange::get(glp);
    GSpline curve_1 = interpolate(tau, waypoints, *basis_lagrange);
    int counter = 0;
    while (ros::ok() and counter < 30) {
      pub.publish(gspline_to_msg(curve_1));
      ros::spinOnce();
      ros::WallDuration(0.01).sleep();
      if (gspline_received) {
        // EXPECT_TRUE(*gspline_received == curve_1);
      }
      counter++;
    }
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
