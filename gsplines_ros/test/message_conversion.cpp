

#include <cstddef>
#include <eigen3/Eigen/Core>
#include <gsplines/Basis/BasisLagrange.hpp>
#include <gsplines/Basis/BasisLegendre.hpp>
#include <gsplines/Tools.hpp>
#include <gsplines_ros/gsplines_ros.hpp>
#include <gtest/gtest.h>

/* Test that the Legendre polynomials respect the recursive relation */
using namespace gsplines::basis;
using namespace gsplines_ros;
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

/* Test that the derivtaive of the Legendre polynomials respect the recursive
 * relation */
int main(int argc, char **argv) {

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
