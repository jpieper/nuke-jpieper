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

import math

RIGHT_FRONT, RIGHT_REAR, LEFT_FRONT, LEFT_REAR = range(4)
LEGS = [RIGHT_FRONT, RIGHT_REAR, LEFT_FRONT, LEFT_REAR]
LEG_PREFIX = ['RF', 'RR', 'LF', 'LR']
_TRAN_TIME_S = 0.1

class Point3D(object):
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __repr__(self):
        return '<Point3D x/y/z %r/%r/%f>' % (self.x, self.y, self.z)

    def __add__(self, other):
        return Point3D(
            self.x + other.x,
            self.y + other.y,
            self.z + other.z)

    def __mul__(self, other):
        return Point3D(
            self.x * other.x,
            self.y * other.y,
            self.z * other.z)

class LegGait(Point3D):
    def __init__(self, x, y, z, r):
        super(LegGait, self).__init__(x, y, z)
        self.r = r

    def copy(self, other):
        self.x = other.x
        self.y = other.y
        self.z = other.z
        self.r = other.r

    def __repr__(self):
        return '<LegGait x/y/z/r=%6.2f/%6.2f/%6.2f/%6.2f>' % (
            self.x, self.y, self.z, self.r)

class Speed(object):
    def __init__(self, x=0.0, y=0.0, r=0.0):
        self.x = x
        self.y = y
        self.r = r

    def is_moving(self):
        return abs(self.x) > 5 or abs(self.y) > 5 or abs(self.r) > 0.05

class LegJoints(object):
    def __init__(self, coxa_rad, femur_rad, tibia_rad):
        self.coxa_rad = coxa_rad
        self.femur_rad = femur_rad
        self.tibia_rad = tibia_rad

    def __repr__(self):
        return '<LegJoints c/f/t=%r/%r/%f>' % (math.degrees(self.coxa_rad),
                                               math.degrees(self.femur_rad),
                                               math.degrees(self.tibia_rad))

class Solver(object):
    DEFAULT_PARAMS = {
        'X_COXA': 100,
        'Y_COXA': 100,
        'L_COXA': 60,
        'L_FEMUR': 40,
        'L_TIBIA': 60,
        }

    def __init__(self, gait, params=DEFAULT_PARAMS):
        self.params = params
        self.gait = gait

        self._body_rot_x = 0.
        self._body_rot_y = 0.
        self._body_rot_z = 0.
        self._body_pos_x = 0.
        self._body_pos_y = 0.

        self._leg_IK = self._lizard_leg_IK

        self._setup_IK()

    def set_speed(self, speed):
        self.gait.set_speed(speed)

    def advance_time(self, advance_s):
        '''Advance time by the given number of seconds.'''
        self.gait.advance_time(advance_s)

    def servo_state(self):
        '''Return the desired current position of each servo at this
        point in time.'''
        servos = self._do_IK()
        result = {}
        for legnum, joints in servos.iteritems():
            prefix = LEG_PREFIX[legnum]
            result[prefix + ' Coxa'] = joints.coxa_rad
            result[prefix + ' Femur'] = joints.femur_rad
            result[prefix + ' Tibia'] = joints.tibia_rad

        return result

    _LEG_SIGN = { RIGHT_FRONT: Point3D(1.0, 1.0, 1.0),
                  RIGHT_REAR: Point3D(-1.0, 1.0, 1.0),
                  LEFT_FRONT: Point3D(1.0, -1.0, 1.0),
                  LEFT_REAR: Point3D(-1.0, -1.0, 1.0) }

    def _setup_IK(self):
        x = 0
        y = 0.75 * (self.params['L_COXA'] + self.params['L_FEMUR'])
        z = 0.75 * self.params['L_TIBIA']
        self._endpoints = {}
        for leg in LEGS:
            self._endpoints[leg] = Point3D(x, y, z) * self._LEG_SIGN[leg]

    def _body_IK(self, point3d, x_disp, y_disp, z_rot):
        cosB = math.cos(self._body_rot_x)
        sinB = math.sin(self._body_rot_x)
        cosG = math.cos(self._body_rot_y)
        sinG = math.sin(self._body_rot_y)
        cosA = math.cos(self._body_rot_z + z_rot)
        sinA = math.sin(self._body_rot_z + z_rot)

        total_x = point3d.x + x_disp + self._body_pos_x
        total_y = point3d.y + y_disp + self._body_pos_y
        total_z = point3d.z

        return Point3D(
            total_x - (total_x * cosG * cosA +
                       total_y * sinB * sinG * cosA +
                       total_z * cosB * sinG * cosA -
                       total_y * cosB * sinA +
                       total_z * sinB * sinA) + self._body_pos_x,
            total_y - (total_x * cosG * sinA +
                       total_y * sinB * sinG * sinA +
                       total_z * cosB * sinG * sinA +
                       total_y * cosB * cosA) + self._body_pos_y,
            total_z - (-total_x * sinG +
                        total_y * sinB * cosG +
                        total_z * cosB * cosG))

    def _lizard_leg_IK(self, point3d):
        print point3d
        # First, make this a 2DOG problem... by solving coxa.
        coxa_rad = math.atan2(point3d.x, point3d.y)
        true_x = (math.sqrt(point3d.x ** 2 + point3d.y ** 2) -
                  self.params['L_COXA'])
        im = math.sqrt(true_x ** 2 + point3d.z ** 2) # length of imaginary leg

        # Get femur angle above horizon...
        q1 = -math.atan2(point3d.z, true_x)
        d1 = (self.params['L_FEMUR'] ** 2 -
              self.params['L_TIBIA'] ** 2 +
              im ** 2)
        d2 = 2 * self.params['L_FEMUR'] * im
        q2 = math.acos(d1 / d2)
        femur_rad = q1 + q2

        # And tibia angle from femur...
        d1 = self.params['L_FEMUR'] ** 2 - im ** 2 + self.params['L_TIBIA'] ** 2
        d2 = 2 * self.params['L_TIBIA'] * self.params['L_FEMUR']
        tibia_rad = math.acos(d1 / d2) - 0.5 * math.pi

        return LegJoints(coxa_rad, femur_rad, tibia_rad)

    def _do_IK(self):
        x_coxa = self.params['X_COXA']
        y_coxa = self.params['Y_COXA']

        result = {}

        gait_offsets = self.gait.gen()
        print
        for leg, gait_offset in zip(LEGS, gait_offsets):
            print LEG_PREFIX[leg], gait_offset
            sign = self._LEG_SIGN[leg]
            req = self._body_IK(self._endpoints[leg] + gait_offset,
                                sign.x * x_coxa,
                                sign.y * y_coxa,
                                gait_offset.r)
            sol = self._leg_IK(sign *
                               (self._endpoints[leg] + gait_offset + req))
            result[leg] = sol

        return result

class Gait(object):
    def __init__(self):
        self._time_s = 0.0
        self._cycle_time_s = 1.0
        self._steps_in_cycle = 8
        self._lift_height = 20
        self._speed = Speed()
        self._gait_leg_number = { }
        self._gaits = { }
        self._old_gaits = { }
        self._leg_time = { }
        for leg in LEGS:
            self._gaits[leg] = LegGait(0.0, 0.0, 0.0, 0.0)
            self._old_gaits[leg] = LegGait(0.0, 0.0, 0.0, 0.0)
        self._last_step = 0

    def advance_time(self, time_s):
        self._time_s += time_s

    def set_speed(self, speed):
        self._speed = speed

    def step(self):
        step_float = (math.fmod(self._time_s, self._cycle_time_s) /
                      self._cycle_time_s * self._steps_in_cycle)
        step_int = int(step_float)
        step_fraction = math.fmod(step_float, 1.0)
        return step_int, step_fraction

    def update_leg_real(self, leg, old_gait, this_leg, step, fraction):
        if not self._speed.is_moving():
            #this_leg.z = 0
            return

        if step == self._gait_leg_number[leg]:
            # Leg up, middle position.
            this_leg.x = (1.0 - fraction) * old_gait.x
            this_leg.y = (1.0 - fraction) * old_gait.y
            this_leg.z = fraction * self._lift_height
            this_leg.r = (1.0 - fraction) * old_gait.r
        elif (step == ((self._gait_leg_number[leg] + 1) %
                       self._steps_in_cycle)):
            # Leg down position. NOTE: dutyFactor = pushSteps / StepsInCycle
            this_leg.x = (old_gait.x +
                          (fraction *
                           self._speed.x *
                           self._cycle_time_s *
                           self._push_steps) / (2 * self._steps_in_cycle))
            this_leg.y = (old_gait.y +
                          (fraction *
                           self._speed.y *
                           self._cycle_time_s *
                           self._push_steps) / (2 * self._steps_in_cycle))
            this_leg.z = (1.0 - fraction) * (self._lift_height)
            this_leg.r = (old_gait.r +
                          (fraction *
                           self._speed.r *
                           self._cycle_time_s *
                           self._push_steps) / (2 * self._steps_in_cycle))
        else:
            # Move body forward
            this_leg.x = (old_gait.x -
                          self._speed.x * fraction * self._cycle_time_s /
                          self._steps_in_cycle)
            this_leg.y = (old_gait.y -
                          self._speed.y * fraction * self._cycle_time_s /
                          self._steps_in_cycle)
            this_leg.z = 0
            this_leg.r = (old_gait.r -
                          self._speed.r * fraction * self._cycle_time_s /
                          self._steps_in_cycle)

    def update_leg(self, leg_num,
                   last_step,
                   step, fraction):
        leg_gait = self._gaits[leg_num]
        old_gait = self._old_gaits[leg_num]
        if step != last_step:
            self.update_leg_real(leg_num, old_gait, leg_gait, last_step, 1.0)
            old_gait.copy(leg_gait)
        self.update_leg_real(leg_num, old_gait, leg_gait, step, fraction)

    def default_gait_gen(self):
        step, fraction = self.step()
        if (step != self._last_step and
            step != (self._last_step + 1) % self._steps_in_cycle):
            raise RuntimeError('Time advanced by more than 1 step')

        for leg in LEGS:
            self.update_leg(leg,
                            self._last_step,
                            step, fraction)

        self._last_step = step

        return [self._gaits[leg] for leg in LEGS]
        return this_leg

class GaitRipple(Gait):
    def __init__(self):
        super(GaitRipple, self).__init__()

        self._gait_leg_number[RIGHT_FRONT] = 0
        self._gait_leg_number[LEFT_REAR] = 2
        self._gait_leg_number[LEFT_FRONT] = 4
        self._gait_leg_number[RIGHT_REAR] = 6
        self._push_steps = 6
        self._steps_in_cycle = 8
        self._cycle_time_s = self._steps_in_cycle * _TRAN_TIME_S
        self.gen = self.default_gait_gen

class GaitRippleSmooth(Gait):
    def __init__(self):
        super(GaitRippleSmooth, self).__init__(self)

        self._gait_leg_number[RIGHT_FRONT] = 0
        self._gait_leg_number[LEFT_REAR] = 4
        self._gait_leg_number[LEFT_FRONT] = 8
        self._gait_leg_number[RIGHT_REAR] = 12
        self._push_steps = 12
        self._steps_in_cycle = 16
        self._cycle_time_s = self._steps_in_cycle * _TRAN_TIME_S
        self.gen = self.default_gait_gen
