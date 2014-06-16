#!/usr/bin/python

"""
  Nuke: The Nearly Universal Kinematics Engine
  Copyright (c) 2009-2010 Michael E. Ferguson.  All right reserved.
  Copyright 2014 Josh Pieper, jjp@pobox.com.

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software Foundation,
  Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
"""

import unittest

import gaits

class SolverTest(unittest.TestCase):
    def setUp(self):
        self.gait = gaits.GaitRipple()
        self.solver = gaits.Solver(self.gait)

    def test_basic(self):
        self.solver.set_speed(gaits.Speed(30, 0, 0))
        print self.solver.servo_state()
        self.solver.advance_time(0.02)
        print self.solver.servo_state()

class BasicGaitTest(unittest.TestCase):
    def is_leg_close(self, left, right):
        if abs(left.x - right.x) > .1:
            return False
        if abs(left.y - right.y) > .1:
            return False
        if abs(left.z - right.z) > .1:
            return False
        if abs(left.r - right.r) > 0.01:
            return False
        return True

    def are_legs_close(self, left, right):
        for l, r in zip(left, right):
            if not self.is_leg_close(l, r):
                return False
        return True

    def expect_legs(self, expected):
        actual = self.gait.gen()
        self.assertEqual(len(actual), len(expected))

        if not self.are_legs_close(actual, expected):
            self.assertTrue(
                False, msg='actual=%r expected=%r' % (actual, expected))


    def test_stationary(self):
        self.gait = gaits.GaitRipple()

        # When stationary, all legs should be at 0, 0, 0.
        self.expect_legs([gaits.LegGait(0, 0, 0, 0) for x in range(4)])

        # This should be the case even after advancing time arbitrary
        # amounts.
        for x in range(100):
            self.gait.advance_time(0.01)
            self.expect_legs([gaits.LegGait(0, 0, 0, 0) for x in range(4)])

    def test_basic_moving(self):
        self.gait = gaits.GaitRipple()
        self.gait.set_speed(gaits.Speed(20, 0, 0))

        self.expect_legs([gaits.LegGait(0, 0, 0, 0),
                          gaits.LegGait(0, 0, 0, 0),
                          gaits.LegGait(0, 0, 0, 0),
                          gaits.LegGait(0, 0, 0, 0)])

        # Move just a little bit forward in time, so that the only
        # change are the legs which are still on the ground.
        self.gait.advance_time(0.01)

        self.expect_legs([gaits.LegGait(0, 0, -2.7, 0),
                          gaits.LegGait(-0.2, 0, 0, 0),
                          gaits.LegGait(-0.2, 0, 0, 0),
                          gaits.LegGait(-0.2, 0, 0, 0)])

        # Now move a little less than a full step forward.
        self.gait.advance_time(0.055)

        self.expect_legs([gaits.LegGait(0, 0, -17.7, 0),
                          gaits.LegGait(-1.3, 0, 0, 0),
                          gaits.LegGait(-1.3, 0, 0, 0),
                          gaits.LegGait(-1.3, 0, 0, 0)])

        # Now move such that we're looking at the next step.
        self.gait.advance_time(0.01)

        self.expect_legs([gaits.LegGait(0.5, 0, -15.5, 0),
                          gaits.LegGait(-1.5, 0, 0, 0),
                          gaits.LegGait(-1.5, 0, 0, 0),
                          gaits.LegGait(-1.5, 0, 0, 0)])

        # Now move close to the end of this step.
        self.gait.advance_time(0.055)

        self.expect_legs([gaits.LegGait(3.84, 0, -0.55, 0),
                          gaits.LegGait(-2.6, 0, 0, 0),
                          gaits.LegGait(-2.6, 0, 0, 0),
                          gaits.LegGait(-2.6, 0, 0, 0)])

        # Finally, advance a little into the next step.
        self.gait.advance_time(0.01)

        self.expect_legs([gaits.LegGait(3.8, 0, 0, 0),
                          gaits.LegGait(-2.8, 0, 0, 0),
                          gaits.LegGait(-2.8, 0, 0, 0),
                          gaits.LegGait(-2.32, 0, -2.2, 0)])



if __name__ == '__main__':
    unittest.main()
