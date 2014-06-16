#!/usr/bin/env python

"""
  PyPose: Bioloid pose system for arbotiX robocontroller
  Copyright (c) 2014 Josh Pieper, jjp@pobox.com
  Copyright (c) 2009,2010 Michael E. Ferguson.  All right reserved.

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

import math, time, sys, serial
import wx

from ax12 import *
import gazebo_driver
sys.path.append('tools/ik')
import gaits

width = 300

class Servo(object):
    def __init__(self, num, sign, neutral=512):
        self.num = num
        self.sign = sign
        self.neutral = neutral

    def rad_to_servo(self, rads):
        val = (rads*100)/51 * 100;
        return self.neutral + self.sign * int(val)

class LiveGait(wx.Frame):
    TIMER_ID = 100

    def __init__(self, parent, port, debug = False):
        wx.Frame.__init__(self, parent, -1, "Live GaitCheck",
                          style = wx.DEFAULT_FRAME_STYLE &
                          ~ (wx.RESIZE_BORDER | wx.MAXIMIZE_BOX))
        self.port = port
        self.servo_mapping = {
            'LF Coxa' : Servo(10, -1),
            'LF Femur': Servo(11, 1),
            'LF Tibia': Servo(12, 1, 819),
            'RF Coxa': Servo(1, 1),
            'RF Femur': Servo(2, 1),
            'RF Tibia': Servo(3, 1, 819),
            'LR Coxa': Servo(7, 1),
            'LR Femur': Servo(8, 1),
            'LR Tibia': Servo(9, 1, 819),
            'RR Coxa': Servo(4, -1),
            'RR Femur': Servo(5, 1),
            'RR Tibia': Servo(6, 1, 819),
            }


        sizer = wx.GridBagSizer(10,10)

        self.drive = wx.Panel(self,size=(width,width-20))
        self.drive.SetBackgroundColour('WHITE')
        self.drive.Bind(wx.EVT_MOTION, self.onMove)
        wx.StaticLine(self.drive, -1, (width/2, 0), (1,width),
                      style=wx.LI_VERTICAL)
        wx.StaticLine(self.drive, -1, (0, width/2), (width,1))
        sizer.Add(self.drive,(0,0),wx.GBSpan(2,1),wx.EXPAND|wx.ALL,5)
        self.forward = 0
        self.turn = 0

        # Selection for horizontal movement
        horiz = wx.StaticBox(self, -1, 'Horizontal Movement')
        horiz.SetFont(wx.Font(10, wx.DEFAULT, wx.NORMAL, wx.BOLD))
        horizBox = wx.StaticBoxSizer(horiz,orient=wx.VERTICAL)

        self.selTurn = wx.RadioButton(self, -1, 'Turn', style=wx.RB_GROUP)
        horizBox.Add(self.selTurn)
        self.selStrafe = wx.RadioButton(self, -1, 'Strafe')
        horizBox.Add(self.selStrafe)
        sizer.Add(horizBox, (0,1), wx.GBSpan(1,1), wx.EXPAND|wx.TOP|wx.RIGHT,5)

        # Body rotations
        body = wx.StaticBox(self, -1, 'Body Movement')
        body.SetFont(wx.Font(10, wx.DEFAULT, wx.NORMAL, wx.BOLD))
        bodyBox = wx.StaticBoxSizer(body,orient=wx.VERTICAL)
        bodySizer = wx.GridBagSizer(5,5)

        bodySizer.Add(wx.StaticText(self, -1, "Pan:"),(0,0), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)
        self.pan = wx.Slider(self, -1, 0, -100, 100, wx.DefaultPosition, (200, -1), wx.SL_HORIZONTAL | wx.SL_LABELS)
        bodySizer.Add(self.pan,(0,1))
        bodySizer.Add(wx.StaticText(self, -1, "Tilt:"),(1,0), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)
        self.tilt = wx.Slider(self, -1, 0, -100, 100, wx.DefaultPosition, (200, -1), wx.SL_HORIZONTAL | wx.SL_LABELS)
        bodySizer.Add(self.tilt,(1,1))
        bodySizer.Add(wx.StaticText(self, -1, "Roll:"),(2,0), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)
        self.roll = wx.Slider(self, -1, 0, -100, 100, wx.DefaultPosition, (200, -1), wx.SL_HORIZONTAL | wx.SL_LABELS)
        self.roll.Disable()
        bodySizer.Add(self.roll,(2,1))
        bodyBox.Add(bodySizer)

        sizer.Add(bodyBox, (1,1), wx.GBSpan(1,1), wx.EXPAND|wx.BOTTOM|wx.RIGHT,5)

        # timer for output
        self.timer = wx.Timer(self, self.TIMER_ID)
        self.timer.Start(20)
        wx.EVT_CLOSE(self, self.onClose)
        wx.EVT_TIMER(self, self.TIMER_ID, self.onTimer)

        self.SetSizerAndFit(sizer)
        self.Show(True)

        self.gait = gaits.GaitRipple()

        class FakeGait(object):
            def __init__(self):
                self.data = [gaits.LegGait(0, 0, 0, 0) for x in range(4)]

            def gen(self):
                return self.data

            def advance_time(self, time):
                pass

            def set_speed(self, foo):
                pass

        #self.gait = FakeGait()
        self.solver = gaits.Solver(self.gait)


    def onClose(self, event):
        self.timer.Stop()
        self.Destroy()

    def onMove(self, event=None):
        if event.LeftIsDown():
            pt = event.GetPosition()
            self.forward = 0.75 * ((width/2)-pt[1])
            self.turn = (pt[0]-(width/2))/2
        else:
            self.forward = 0
            self.turn = 0
            pass

    def onTimer(self, event=None):
        # configure output
        turn = 0
        strafe = 0
        if self.selStrafe.GetValue():
            strafe = self.turn
        else:
            turn = math.radians(self.turn)

        self.solver.set_speed(gaits.Speed(self.forward, strafe, turn))
        self.solver.advance_time(0.02)
        servos = self.solver.servo_state()

        for servo_name, value_rad in servos.iteritems():
            servo = self.servo_mapping[servo_name]
            value_int = servo.rad_to_servo(value_rad)
            self.port.setReg(servo.num, P_GOAL_POSITION_L,
                             [value_int % 256, value_int >> 8])


if __name__ == "__main__":
    # commander.py <serialport>
    port = gazebo_driver.GazeboDriver()

    app = wx.PySimpleApp()
    frame = LiveGait(None, port, True)
    app.MainLoop()
