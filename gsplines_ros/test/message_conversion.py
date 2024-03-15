#!/usr/bin/env python3
# import sys
import unittest
# import numpy as np
# from gsplines.basis import BasisLegendre
# from gsplines.basis import BasisLagrange
# from gsplines import Interpolator

# from gsplines_ros import gspline_msg_to_gspline, gspline_to_msg


class MessageConversion(unittest.TestCase):

    def test_one_equals_one(self):
        # basis_dim = 8
        # basis = BasisLegendre(basis_dim)
        # dim = np.random.randint(1, 10)
        # intervals = np.random.randint(3, 6)
        # waypoints = np.random.rand(intervals+1, dim)
        # interval_lengths = 1.0 + np.random.rand(intervals) * 2.0
        # inter = Interpolator(dim, intervals, basis)
        # res = inter.interpolate(interval_lengths, waypoints)

        self.assertTrue(True)


#


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('gsplines_ros', 'message_conversion_py', MessageConversion)
