# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import eventlet
import math
import sys

from ax12 import *

import pygazebo
from pygazebo.msg import joint_cmd_pb2

SERVO_NAMES = reduce(lambda a, b: a + b,
                     [ [ 'mj_mech::coxa_hinge%d' % x,
                         'mj_mech::femur_hinge%d' % x,
                         'mj_mech::tibia_hinge%d' % x ]
                       for x in range(4) ])

class Servo(object):
    def __init__(self, parent, ident):
        self.parent = parent
        self.ident = ident
        self.register_file = [0x00] * 64

    def execute(self, ins, params):
        # TODO
        return 0

    def setReg(self, regstart, values):
        reg = regstart
        for value in values:
            self.register_file[reg] = value
            reg += 1

        if regstart == P_GOAL_POSITION_L:
            self.register_file[P_PRESENT_POSITION_L] = self.register_file[P_GOAL_POSITION_L]
            self.register_file[P_PRESENT_POSITION_H] = self.register_file[P_GOAL_POSITION_H]
            goal = (self.register_file[P_GOAL_POSITION_H] << 8 |
                    self.register_file[P_GOAL_POSITION_L])
            angle = ((goal - 512.0) / 512) * math.radians(149.610)
            #print 'writing goal:', self.ident, angle
            self.parent.joint_cmd.name = SERVO_NAMES[self.ident]
            self.parent.joint_cmd.position.target = angle
            self.parent.joint_publisher.publish(self.parent.joint_cmd)

        return 0

    def getReg(self, regstart, rlength):
        if rlength == 1:
            return self.register_file[regstart]
        return self.register_file[regstart:regstart+rlength]

    def update_joint(self, joint_cmd):
        if not joint_cmd.HasField('position'):
            return
        if not joint_cmd.position.HasField('target'):
            return
        angle = joint_cmd.position.target
        value = int((angle / (3 * math.pi / 4)) * 512 + 512)
        self.register_file[P_PRESENT_POSITION_L] = value & 0xff
        self.register_file[P_PRESENT_POSITION_H] = (value >> 8) & 0xff

class _DummyPort(object):
    def close(self):
        pass

    def write(self, data):
        pass

    def inWaiting(self):
        return 0

class GazeboDriver(object):
    '''This class emulates an AX12 using the Driver interface.  Final
    positions are sent over the gazebo_client to a simulation.'''

    def __init__(self):
        self.manager = pygazebo.Manager()
        self.hasInterpolation = False
        self.direct = False
        self.servos = [Servo(self, x) for x in range(12)]
        self.ser = _DummyPort()

        self.joint_publisher = self.manager.advertise(
            '/gazebo/default/mj_mech/joint_cmd',
            'gazebo.msgs.JointCmd')

        self.joint_subscriber = self.manager.subscribe(
            '/gazebo/default/mj_mech/joint_cmd',
            'gazebo.msgs.JointCmd',
            self._receive_joint_cmd)

        self.joint_cmd = joint_cmd_pb2.JointCmd()
        self.joint_cmd.axis = 0
        self.joint_cmd.position.target = 0
        self.joint_cmd.position.p_gain = 10.0
        self.joint_cmd.position.i_gain = 2.0
        self.joint_cmd.position.d_gain = 0.2

    def execute(self, index, ins, params):
        eventlet.sleep()
        if (index < 1 or index >= len(self.servos)):
            return 0
        return self.servos[index - 1].execute(ins, params)

    def setReg(self, index, regstart, values):
        eventlet.sleep()
        return self.servos[index - 1].setReg(regstart, values)

    def getReg(self, index, regstart, rlength):
        eventlet.sleep()
        return self.servos[index - 1].getReg(regstart, rlength)

    def _receive_joint_cmd(self, data):
        print 'got joint cmd'
        joint_cmd = joint_cmd_pb2.JointCmd.FromString(data)
        # Update the present position for the servo if we have it.
        index = SERVO_NAMES.index(joint_cmd.name)
        if index < 0:
            return
        self.servos[index].update_joint(joint_cmd)
