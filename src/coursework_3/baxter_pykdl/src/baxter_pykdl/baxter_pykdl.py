#!/usr/bin/python

# Copyright (c) 2013-2014, Rethink Robotics
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

import numpy as np
import PyKDL

import rospy

import baxter_interface

from baxter_kdl.kdl_parser import kdl_tree_from_urdf_model
from urdf_parser_py.urdf import URDF
   


class baxter_kinematics(object):
    """
    Baxter Kinematics with PyKDL
    """
    def __init__(self, limb):
        # Baxter Interface Limb Instances
        self._limb_interface = baxter_interface.Limb(limb)
        self._joint_names = self._limb_interface.joint_names()
        self._num_jnts = len(self._joint_names)

        self._baxter = URDF.from_parameter_server(key='robot_description')
        self._kdl_tree = kdl_tree_from_urdf_model(self._baxter)
        self._base_link = self._baxter.get_root()
        self._tip_link = limb + '_gripper'
        self._tip_frame = PyKDL.Frame()
        self._arm_chain = self._kdl_tree.getChain(self._base_link,
                                                  self._tip_link)
        self._segment_names = [self._arm_chain.getSegment(idx).getName() for idx in xrange(self._arm_chain.getNrOfSegments())]
        print self._segment_names
        # KDL Solvers
        # Forward kinematics
        self._fk_p_kdl = PyKDL.ChainFkSolverPos_recursive(self._arm_chain)
        self._fk_v_kdl = PyKDL.ChainFkSolverVel_recursive(self._arm_chain)
        
        # inverse kinematics
        self._ik_v_kdl = PyKDL.ChainIkSolverVel_pinv(self._arm_chain)
        self._ik_p_kdl = PyKDL.ChainIkSolverPos_NR(self._arm_chain,
                                                   self._fk_p_kdl,
                                                   self._ik_v_kdl)
        self._jac_kdl = PyKDL.ChainJntToJacSolver(self._arm_chain)
        
        self.grav_vector = PyKDL.Vector(0, 0, -9.81)
        
        self._dyn_kdl = PyKDL.ChainDynParam(self._arm_chain,
                                            self.grav_vector)

    def print_robot_description(self):
        nf_joints = 0
        for j in self._baxter.joints:
            if j.type != 'fixed':
                nf_joints += 1
        print "URDF non-fixed joints: %d;" % nf_joints
        print "URDF total joints: %d" % len(self._baxter.joints)
        print "URDF links: %d" % len(self._baxter.links)
        print "KDL joints: %d" % self._kdl_tree.getNrOfJoints()
        print "KDL segments: %d" % self._kdl_tree.getNrOfSegments()
    
    def current_robot_state(self):
        q = np.array([self.kdl_to_list(self.joints_to_kdl('positions'))]).T
        q_dot = np.array([self.kdl_to_list(self.joints_to_vel_kdl().qdot)]).T
        tau = np.array([self.kdl_to_list(self.joints_to_kdl('torques'))]).T
        return q, q_dot, tau

    def print_kdl_chain(self):
        for idx in xrange(self._arm_chain.getNrOfSegments()):
            print '* ' + self._arm_chain.getSegment(idx).getName()

    def joints_to_kdl(self, type, values=None, n_jnts=None):
        if n_jnts is None:
            nj = self._num_jnts
        else:
            nj = n_jnts
        kdl_array = PyKDL.JntArray(nj)
            
        if values is None:
            if type == 'positions':
                cur_type_values = self._limb_interface.joint_angles()
            elif type == 'velocities':
                cur_type_values = self._limb_interface.joint_velocities()
            elif type == 'torques':
                cur_type_values = self._limb_interface.joint_efforts()
        else:
            cur_type_values = values

        for idx, name in enumerate(self._joint_names):
            if idx >= nj:
                break
            try:
                kdl_array[idx] = cur_type_values[name]
            except Exception as e:
                kdl_array[idx] = cur_type_values[idx]
            if type == 'velocities':
                kdl_array = PyKDL.JntArrayVel(kdl_array)
        return kdl_array
    
    def joints_to_vel_kdl(self, q=None, qdot=None):
        q_kdl = PyKDL.JntArray(self._num_jnts)
        qdot_kdl = PyKDL.JntArray(self._num_jnts)
        if q is None:
            q = self._limb_interface.joint_angles()
        if qdot is None:
            qdot = self._limb_interface.joint_velocities()
        for idx, name in enumerate(self._joint_names):
            try:
                q_kdl[idx] = q[name]
                qdot_kdl[idx] = qdot[name]
            except Exception as e:
                q_kdl[idx] = q[idx]
                qdot_kdl[idx] = qdot[idx]
        kdl_array = PyKDL.JntArrayVel(q_kdl, qdot_kdl)
        return kdl_array
    
    def kdl_to_list(self, q):
        if q == None:
            return None
        return [q[i] for i in range(q.rows())]

    def kdl_to_mat(self, data):
        mat =  np.mat(np.zeros((data.rows(), data.columns())))
        for i in range(data.rows()):
            for j in range(data.columns()):
                mat[i,j] = data[i,j]
        return mat

    def forward_position_kinematics(self,joint_values=None):
        end_frame = PyKDL.Frame()
        self._fk_p_kdl.JntToCart(self.joints_to_kdl('positions',joint_values),
                                 end_frame)
        pos = end_frame.p
        rot = PyKDL.Rotation(end_frame.M)
        rot = rot.GetQuaternion()
        return np.array([pos[0], pos[1], pos[2],
                         rot[0], rot[1], rot[2], rot[3]]).reshape((self._num_jnts, 1))

    def forward_velocity_kinematics(self,joint_velocities=None):
        end_frame = PyKDL.FrameVel()
        self._fk_v_kdl.JntToCart(self.joints_to_kdl('velocities',joint_velocities),
                                 end_frame)
        return end_frame.GetTwist()

    def inverse_kinematics(self, position, orientation=None, seed=None):
        ik = PyKDL.ChainIkSolverVel_pinv(self._arm_chain)
        pos = PyKDL.Vector(position[0], position[1], position[2])
        pos_only = True
        try:
            if orientation != None:
                pos_only = False
                rot = PyKDL.Rotation()
                rot = rot.Quaternion(orientation[0], orientation[1],
                                     orientation[2], orientation[3])
        except Exception as e:
            pos_only = False
            rot = PyKDL.Rotation()
            rot = rot.Quaternion(orientation[0], orientation[1],
                                 orientation[2], orientation[3])
        # Populate seed with current angles if not provided
        seed_array = PyKDL.JntArray(self._num_jnts)
        if seed != None:
            seed_array.resize(len(seed))
            for idx, jnt in enumerate(seed):
                seed_array[idx] = jnt
        else:
            seed_array = self.joints_to_kdl('positions')

        # Make IK Call
        if not pos_only:
            goal_pose = PyKDL.Frame(rot, pos)
        else:
            goal_pose = PyKDL.Frame(pos)
        result_angles = PyKDL.JntArray(self._num_jnts)

        if self._ik_p_kdl.CartToJnt(seed_array, goal_pose, result_angles) >= 0:
            result = np.array(list(result_angles)).reshape((self._num_jnts, 1))
            return result
        else:
            return None
    
    def jacobian(self,joint_values=None):
        jacobian = PyKDL.Jacobian(self._num_jnts)
        self._jac_kdl.JntToJac(self.joints_to_kdl('positions',joint_values), jacobian)
        return self.kdl_to_mat(jacobian)

    def jacobian_transpose(self,joint_values=None):
        return self.jacobian(joint_values).T

    def jacobian_pseudo_inverse(self,joint_values=None):
        return np.linalg.pinv(self.jacobian(joint_values))

    def inertia(self,joint_values=None):
        """ calculates inertia matrix """
        inertia = PyKDL.JntSpaceInertiaMatrix(self._num_jnts)
        self._dyn_kdl.JntToMass(self.joints_to_kdl('positions',joint_values), inertia)
        return self.kdl_to_mat(inertia)
    
    def gravity(self,joint_values=None):
        """ calculates gravity torques """
        gravity = PyKDL.JntArray(self._num_jnts)
        self._dyn_kdl.JntToGravity(self.joints_to_kdl('positions',joint_values), gravity)
        gravity_torques = np.array([gravity[i] for i in range(gravity.rows())]).reshape((self._num_jnts, 1))
        return gravity_torques
        
    def coriolis(self,joint_values=None, joint_velocities=None):
        """ calculates coriolis torques """
        coriolis = PyKDL.JntArray(self._num_jnts)
        self._dyn_kdl.JntToCoriolis(self.joints_to_kdl('positions',joint_values),
                                    self.joints_to_vel_kdl(joint_values, joint_velocities).qdot,
                                    coriolis)
        coriolis_torques = np.array([coriolis[i] for i in range(coriolis.rows())]).reshape((self._num_jnts, 1))
        return coriolis_torques

    def cart_inertia(self, joint_values=None):
        js_inertia = self.inertia(joint_values)
        jacobian = self.jacobian(joint_values)
        return np.linalg.inv(jacobian * np.linalg.inv(js_inertia) * jacobian.T)

