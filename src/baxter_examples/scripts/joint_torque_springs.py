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
import scipy.linalg as la

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

        self.phi_prev = 0
        self.theta_prev = 0

        # pendululm controller gains (from MATLAB)
        self.K = np.array([150, 0, 20, 0])
        # self.K = np.array([150, -10, 20, 5])

        # operational space controller gains
        # self.Kp = 10*np.diag([0.0, 0.0, 10.0, 0.1, 0.1, 0.1]) # control everything except x and y position
        self.Kp = np.diag([0.0, 0.0, 10.0, 0.0, 0.0, 0.0]) # control everything except x and y position
        # self.Kd = np.diag([0.0, 0.0, 5.0, 0.1, 0.1, 0.1]) # control everything except x and y velocity
        self.Kd = np.diag([0.0, 0.0, 5.0, 0.0, 0.0, 0.0]) # control everything except x and y velocity

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

    def pend_cb(self, msg):
        self.theta = -msg.pose.orientation.x
        self.phi = -msg.pose.orientation.y
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

        sigmoid = lambda x: (1.0/(1 + np.exp(-15*x)) - 1)

        # time
        now = rospy.Time.now()
        dt = (now - self.t_prev).to_sec()
        if dt < 1e-6:
            dt = 0.1

        xi = self.calc_xi(q)

        # Calculate dynamic params
        M = bld.M(self.parms,q)
        c = bld.c(self.parms, q, qd)
        J = blk.J[6](q)
        g = bld.g(self.parms, q)

        th = la.norm(xi[3:])
        v = xi[3:]/th
        skew_sym = lambda u: np.array([[0, -u[2], u[1]],[u[2], 0, -u[0]],[-u[1], u[0], 0]])

        T = np.zeros((6,6))
        T[:3,:3] = np.eye(3)
        T[3:,3:] = la.inv(np.eye(3) - skew_sym(v)*(1 - np.cos(th))/th + skew_sym(v).dot(skew_sym(v))*(th - np.sin(th))/th)
        

        JA = T.dot(J)

        Jdot = np.zeros((6,7))
        Jdot = (JA - self.J_prev)/dt
        Jdotqd = np.dot(Jdot,qd)
        self.J_prev = JA
        self.t_prev = now

        # compensate for mass of pendulum
        F = np.zeros(6)
        F[2] = 9.81*0.326

        xi_dot = JA.dot(qd)

        #======================================================================
        # pendulum controller (outer loop)
        # x = [theta, d, omega, v]
        #======================================================================

        #----------------------------------------------------------------------
        # x direction
        #----------------------------------------------------------------------

        x_xaxis = np.zeros(4)
        x_xaxis[0] = self.theta # TODO check sign here
        x_xaxis[1] = xi[0]
        x_xaxis[3] = xi_dot[0]
        x_xaxis[2] = (self.theta - self.theta_prev) / dt
        self.theta_prev = self.theta

        xdes_xaxis = np.zeros(4)
        xdes_xaxis[1] = self.start_xi[0]
        # TODO allow offset in position (xdes_xaxis[1])

        e_x = xdes_xaxis - x_xaxis

        acc_x = self.K.dot(e_x)

        #----------------------------------------------------------------------
        # y direction
        #----------------------------------------------------------------------

        x_yaxis = np.zeros(4)
        x_yaxis[0] = self.phi # TODO check sign here
        x_yaxis[1] = xi[1]
        x_yaxis[3] = xi_dot[1]
        x_yaxis[2] = (self.phi - self.phi_prev) / dt
        self.phi_prev = self.phi

        xdes_yaxis = np.zeros(4)
        xdes_yaxis[1] = self.start_xi[1]
        # TODO allow offset in position (xdes_yaxis[1])

        e_y = xdes_yaxis - x_yaxis

        acc_y = self.K.dot(e_y)

        #======================================================================
        # robot arm controller (inner loop)
        #======================================================================

        # compute desired pose and derivatives
        xi_ddot_des = np.zeros(6)
        xi_ddot_des[0] = acc_x
        xi_ddot_des[1] = acc_y

        xi_dot_des = np.zeros(6)

        xi_des = np.copy(self.start_xi)


        # compute torques
        rospy.logerr(self.sub_xi(xi_des,xi))
        v = np.linalg.pinv(JA).dot(xi_ddot_des + self.Kp.dot(self.sub_xi(xi_des,xi)) + self.Kd.dot(xi_dot_des - xi_dot) - Jdotqd) # TODO we can't just subtract xi_des and xi, can we?
        tau_comp = M.dot(v)  + c + J.T.dot(F)

        if abs(self.phi) > 30*np.pi/180:
            tau_comp = np.zeros(7)
        if abs(self.theta) > 30*np.pi/180:
            tau_comp = np.zeros(7)
        if la.norm((xi_des - xi)[:3]) > 5:
            tau_comp = np.zeros(7)


        # calculate current forces

        tau = {'left_s0': tau_comp[0], 'left_s1': tau_comp[1], 'left_e0': tau_comp[2],
                'left_e1': tau_comp[3], 'left_w0': tau_comp[4], 'left_w1': tau_comp[5],
                'left_w2': tau_comp[6]}

        for joint in self._start_angles.keys():
            # spring portion
            cmd[joint] = tau[joint]

            e_j = self._start_angles[joint] - cur_pos[joint]
            cmd[joint] += self._springs[joint] * e_j

            cmd[joint] -= self._damping[joint] * cur_vel[joint]

        # command new joint torques
        self._limb.set_joint_torques(cmd)

    def move_to_neutral(self):
        """
        Moves the limb to neutral location.
        """
        self._limb.move_to_neutral()

    def calc_xi(self, q):
        skew_sym = lambda u: np.array([[0, -u[2], u[1]],[u[2], 0, -u[0]],[-u[1], u[0], 0]])
        expu = lambda u, th: np.eye(3) + np.sin(th)*skew_sym(u) + (1 - np.cos(th))*skew_sym(u).dot(skew_sym(u))

        # current position and velocity
        T = blk.FK[6](q)
        val, vec = la.eig(T[:3,:3])
        idx = np.where(np.imag(val) == 0)[0]
        th = np.arccos((np.trace(T[:3,:3]) - 1)/2)


        if len(idx) > 1:
            th = 0

        # Check to find sign of angle
        if np.allclose(T[:3,:3], expu(np.real(vec[:,idx[0]]),th)):
            th = th
        else:
            th = -th

        xi = np.zeros(6) # TODO populate this
        xi[:3] = T[:3,3].flatten()
        xi[3:] = th*np.real(vec[:,idx[0]]).flatten()
        return xi

    def sub_xi(self, xi1, xi2):
        dxi = np.zeros(6)
        dxi[:3] = xi1[:3] - xi2[:3]

        skew_sym = lambda u: np.array([[0, -u[2], u[1]],[u[2], 0, -u[0]],[-u[1], u[0], 0]])
        expu = lambda u, th: np.eye(3) + np.sin(th)*skew_sym(u) + (1 - np.cos(th))*skew_sym(u).dot(skew_sym(u))

        th1 = la.norm(xi1[3:])
        if abs(th1) < 1e-8:
            v1 = np.zeros(3)
        else:
            v1 = xi1[3:]/th1

        th2 = la.norm(xi2[3:])
        if abs(th2) < 1e-8:
            v2 = np.zeros(3)
        else:
            v2 = xi2[3:]/th2

        R1 = expu(v1,th1)
        R2 = expu(v2,th2)
        R = R2.T.dot(R1)

        v_skew = la.logm(R)

        dxi[3:] = np.array([v_skew[2,1], -v_skew[2,0], v_skew[1,0]])
        return dxi


    def start_control(self):
        """
        Boots up controller
        """
        # record initial joint angles
        self._start_angles = self._limb.joint_angles()

        q = np.zeros(7)
        q[0] = self._start_angles['left_s0']
        q[1] = self._start_angles['left_s1']
        q[2] = self._start_angles['left_e0']
        q[3] = self._start_angles['left_e1']
        q[4] = self._start_angles['left_w0']
        q[5] = self._start_angles['left_w1']
        q[6] = self._start_angles['left_w2']

        self.start_xi = self.calc_xi(q)

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
    rospy.on_shutdown(pc.clean_shutdown)
    pc.start_control()


if __name__ == "__main__":
    main()
