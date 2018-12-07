#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""
Baxter RSDK Joint Torque Example: joint springs
"""

import argparse

import rospy

import numpy as np

import baxter_left_dynamics as bld
import baxter_left_kinematics as blk

from dynamic_reconfigure.server import (
    Server,
)
from baxter_left_dyn_params import params as parms

from std_msgs.msg import (
    Empty,
)

from geometry_msgs.msg import (
    PoseStamped,
)

import baxter_interface

from baxter_examples.cfg import (
    JointSpringsExampleConfig,
)
from baxter_interface import CHECK_VERSION


class PendulumControl(object):
    """
    Virtual Joint Springs class for torque example.

    @param limb: limb on which to run joint springs example
    @param reconfig_server: dynamic reconfigure server

    JointSprings class contains methods for the joint torque example allowing
    moving the limb to a neutral location, entering torque mode, and attaching
    virtual springs.
    """
    def __init__(self, limb, reconfig_server):
        self._dyn = reconfig_server

        # control parameters
        self._rate = 1000.0  # Hz
        self._missed_cmds = 20.0  # Missed cycles before triggering timeout

        # create our limb instance
        self._limb = baxter_interface.Limb(limb)

        # initialize parameters
        self._springs = dict()
        self._damping = dict()
        self._start_angles = dict()

        # dynamic params for baxter
        self.parms = parms

        # J_prev
        self.J_prev = np.zeros((6,7))
        self.t_prev = rospy.Time.now()

        self.phi = 0
        self.theta = 0
        self.x_offset = 0
        self.y_offset = 0

        # Create pendulum subscriber
        self._sub_pend = rospy.Subscriber("/robot/pendulum/error_pose", PoseStamped, self.pend_cb)

        # create cuff disable publisher
        cuff_ns = 'robot/limb/' + limb + '/suppress_cuff_interaction'
        self._pub_cuff_disable = rospy.Publisher(cuff_ns, Empty, queue_size=1)

        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")

    def pend_cb_(self, msg):
        self.phi = msg.pose.orientation.x
        self.theta = msg.pose.orientation.y
        self.x_offset = msg.pose.position.x
        self.y_offset = msg.pose.position.y

    def _update_parameters(self):
        for joint in self._limb.joint_names():
            self._springs[joint] = self._dyn.config[joint[-2:] +
                                                    '_spring_stiffness']
            self._damping[joint] = self._dyn.config[joint[-2:] +
                                                    '_damping_coefficient']

    def _update_forces(self):
        """
        Calculates the current angular difference between the start position
        and the current joint positions applying the joint torque spring forces
        as defined on the dynamic reconfigure server.
        """
        # get latest spring constants
        self._update_parameters()

        # disable cuff interaction
        self._pub_cuff_disable.publish()

        # create our command dict
        cmd = dict()
        # record current angles/velocities
        cur_pos = self._limb.joint_angles()
        cur_vel = self._limb.joint_velocities()

        # Pack up vector of q and qdot
        q = np.zeros(7)
        qd = np.zeros(7)

        q[0] = cur_pos['left_s0']
        q[1] = cur_pos['left_s1']
        q[2] = cur_pos['left_e0']
        q[3] = cur_pos['left_e1']
        q[4] = cur_pos['left_w0']
        q[5] = cur_pos['left_w1']
        q[6] = cur_pos['left_w2']
        
        qd[0] = cur_vel['left_s0']
        qd[1] = cur_vel['left_s1']
        qd[2] = cur_vel['left_e0']
        qd[3] = cur_vel['left_e1']
        qd[4] = cur_vel['left_w0']
        qd[5] = cur_vel['left_w1']
        qd[6] = cur_vel['left_w2']

        # Calculate dynamic params
        M = bld.M(self.parms,q)
        c = bld.C(self.parms, q, qd)
        J = blk.J[6](q)
        now = rospy.Time.now()
        dt = (now - self.t_prev).to_sec()
        Jdot = (J - self.J_prev)/dt
        Jdotqd = np.dot(Jdot,qd)
        self.J_prev = J
        self.t_prev = now

        # calculate current forces


        for joint in self._start_angles.keys():
            # spring portion
            cmd[joint] = self._springs[joint] * (self._start_angles[joint] -
                                                   cur_pos[joint])
            # damping portion
            cmd[joint] -= self._damping[joint] * cur_vel[joint]
        # command new joint torques
        self._limb.set_joint_torques(cmd)

    def move_to_neutral(self):
        """
        Moves the limb to neutral location.
        """
        self._limb.move_to_neutral()

    def start_control(self):
        """
        Boots up controller
        """
        # record initial joint angles
        self._start_angles = self._limb.joint_angles()

        # set control rate
        control_rate = rospy.Rate(self._rate)

        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot
        # will timeout and disable
        self._limb.set_command_timeout((1.0 / self._rate) * self._missed_cmds)

        # loop at specified rate commanding new joint torques
        while not rospy.is_shutdown():
            if not self._rs.state().enabled:
                rospy.logerr("Joint torque example failed to meet "
                             "specified control rate timeout.")
                break
            self._update_forces()
            control_rate.sleep()

    def clean_shutdown(self):
        """
        Switches out of joint torque mode to exit cleanly
        """
        print("\nExiting example...")
        self._limb.exit_control_mode()
        if not self._init_state and self._rs.state().enabled:
            print("Disabling robot...")
            self._rs.disable()


def main():
    """RSDK Joint Torque Example: Joint Springs
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        '-l', '--limb', dest='limb', required=True, choices=['left', 'right'],
        help='limb on which to attach joint springs'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("rsdk_joint_torque_springs_%s" % (args.limb,))
    dynamic_cfg_srv = Server(JointSpringsExampleConfig,
                             lambda config, level: config)
    pc = PendulumControl(args.limb, dynamic_cfg_srv)
    # register shutdown callback
    rospy.on_shutdown(js.clean_shutdown)
    pc.move_to_neutral()
    pc.start_control()


if __name__ == "__main__":
    main()
