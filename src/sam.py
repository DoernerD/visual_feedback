#!/usr/bin/python
# Copyright 2023 David Doerner (ddorner@kth.se)
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from __future__ import division, print_function
import numpy as np
import tf

class simpleRPMGoal(object):
    def __init__(self,
                 x,
                 y,
                 depth,
                 rpm,
                 tolerance):
        self.x = x
        self.y = y
        self.depth = depth
        self.rpm = rpm
        self.tolerance = tolerance

    @property
    def pos(self):
        return np.array([self.x, self.y, self.depth])

class Sam(object):
    IDLE = "IDLE"
    DRIVE = "DRIVE"
    DIVE = "DIVE"

    def __init__(self,
                 maxRpm = 2000,
                 maxThrustVectorRadians = 0.122,
                 vbsNeutral = 30,
                 lcgNeutral = 70,
                 thrusterKp = 40,
                 thrusterKi = 0,
                 thrusterKd = 1,
                 thrusterKaw = 0,
                 vecHorizontalKp = 5,
                 vecHorizontalKi = 0,
                 vecHorizontalKd = 1,
                 vecHorizontalKaw = 0,
                 vecVerticalKp = 5,
                 vecVerticalKi = 0,
                 vecVerticalKd = 1,
                 vecVerticalKaw = 0,
                 vbsKp = 40,
                 vbsKi = 0,
                 vbsKd = 1,
                 vbsKaw = 0,
                 lcgKp = 60,
                 lcgKi = 0,
                 lcgKd = 6,
                 lcgKaw = 0,
                ):
        """
        A container object that abstracts away ros-related stuff for a nice abstract vehicle
        pose is in NED, x = north, y = east, z = down/depth
        """

        self.maxRpm = maxRpm
        self.maxThrustVectorRadians = maxThrustVectorRadians

        self.thruster.Kp = thrusterKp
        self.thruster.Ki = thrusterKi
        self.thruster.Kd = thrusterKd
        self.thruster.neutral = 0.
        self.thruster.desired = 0.

        self.vecHorizontal.Kp = vecHorizontalKp
        self.vecHorizontal.Ki = vecHorizontalKi
        self.vecHorizontal.Kd = vecHorizontalKd
        self.vecHorizontal.neutral = 0.
        self.vecHorizontal.desired = 0.

        self.vecVertical.Kp = vecVerticalKp
        self.vecVertical.Ki = vecVerticalKi
        self.vecVertical.Kd = vecVerticalKd
        self.vecVertical.neutral = 0.
        self.vecVertical.desired = 0.

        self.vbs.Kp = vbsKp
        self.vbs.Ki = vbsKi
        self.vbs.Kd = vbsKd
        self.vbs.neutral = vbsNeutral
        self.vbs.desired = vbsNeutral

        self.lcg.Kp = lcgKp
        self.lcg.Ki = lcgKi
        self.lcg.Kd = lcgKf
        self.lcg.neutral = lcgNeutral
        self.lcg.desired = lcgNeutral

        self.goal = None
        self.control_mode = Sam.IDLE

        self.pos = np.zeros(3)
        self.ori_rpy = np.zeros(3)

    def _reset_desires(self):
        print("Reset desires to neutral")
        self.thruster.desired = self.thruster.neutral
        self.vecHorizontal.desired = self.vecHorizontal.neutral
        self.vecVertical.desired = self.vecVertical.neutral
        self.vbs.desired = self.vbs.neutral
        self.lcg.desired = self.lcg.neutral
        


    ###############################
    ### Properties for convenience
    ###############################
    @property
    def x(self):
        return self.pos[0]
    @property
    def y(self):
        return self.pos[1]
    @property
    def depth(self):
        return self.pos[2]
    @property
    def roll(self):
        return self.ori_rpy[0]
    @property
    def pitch(self):
        return self.ori_rpy[1]
    @property
    def yaw(self):
        return self.ori_rpy[2]
    @property
    def yaw_vec(self):
        return np.array([np.cos(self.yaw), np.sin(self.yaw)])
    @property
    def ori_quat(self):
        return tf.transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw)
    @property
    def port_rpm(self):
        return self.thruster_rpms[0]
    @property
    def strb_rpm(self):
        return self.thruster_rpms[1]
    @property
    def port_elevon_angle(self):
        return self.elevon_angles[0]
    @property
    def strb_elevon_angle(self):
        return self.elevon_angles[1]
    @property
    def position_error(self):
        return self.goal.pos - self.pos
    @property
    def xy_dist_to_goal(self):
        return geom.euclid_distance(self.goal.pos[:2], self.pos[:2])
    @property
    def depth_to_goal(self):
        return self.goal.depth - self.depth