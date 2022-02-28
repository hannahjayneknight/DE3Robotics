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

import rospy
import numpy as np
from baxter_pykdl import baxter_kinematics


def main():
    rospy.init_node('baxter_kinematics')
    print '*** Baxter PyKDL Kinematics ***\n'
    kin = baxter_kinematics('right')

    print '\n*** Baxter Description ***\n'
    kin.print_robot_description()
    print '\n*** Baxter KDL Chain ***\n'
    kin.print_kdl_chain()
    
    print '\n************** CURRENT BAXTER POINTS **************'
    q, q_dot, tau = kin.current_robot_state()
    # Test points   
    J = kin.jacobian()
    p = kin.forward_position_kinematics()
    p_dot = np.matmul(J, q_dot)
    wrench_external = np.zeros((6, 1))
    
    q_hat = kin.inverse_kinematics(p[:3], p[3:])
    q_dot_hat = np.matmul(np.linalg.pinv(J), p_dot)
    
    gravity_torque = kin.gravity()
    coriolis_torque = kin.coriolis()
    inertia_torque = np.matmul(kin.inertia(), np.zeros((7, 1)))
    external_torque = np.matmul(J.T, wrench_external)
    total_torque = inertia_torque + coriolis_torque + gravity_torque + external_torque
    
    print'Joint Space'
    print'q:\t\n', np.around(q, 3).T
    print'q (inverse kinematics):\n', np.around(q_hat, 3).T
    print'q_dot:\t\n', np.around(q_dot, 3).T
    print'q_dot (inverse kinematics):\n', np.around(q_dot_hat, 3).T
#    print'q_dot_dot:\t\n', np.around(q_dot_dot, 3)
    
    
    print'\nTask Space'
    print'p (forward kinematics):\t\n', np.around(p, 3).T
    print'p_dot (forward kinematics):\t\n', np.around(p_dot, 3).T
    # currently no way to calculate p_dot_dot
    print'external wrench:\n', np.around(wrench_external, 3).T
    
    print'\nDynamics'
    print'gravity torques:\n', np.around(gravity_torque, 3).T
    print'coriolis torques:\n', np.around(coriolis_torque, 3).T
    print'inertia torques:\n', np.around(inertia_torque, 3).T
    print'external torques:\n', np.around(external_torque, 3).T
    print'feed forward torques:\n', np.around(total_torque, 3).T
    
    print '\n************** DEMO POINTS **************'
    # Test points
    q = np.random.uniform(-1, 1, (7, 1))
    q_dot = np.random.uniform(-1, 1, (7, 1))
    q_dot_dot = np.random.uniform(-1, 1, (7, 1))
    
    J = kin.jacobian(q)
    p = kin.forward_position_kinematics(q)
    p_dot = np.matmul(J, q_dot)
    wrench_external = np.random.uniform(-1, 1, (6, 1))
    
    q_hat = kin.inverse_kinematics(p[:3], p[3:])
    q_dot_hat = np.matmul(np.linalg.pinv(J), p_dot)
    
    gravity_torque = kin.gravity(q)
    coriolis_torque = kin.coriolis(q, q_dot)
    inertia_torque = np.matmul(kin.inertia(q), q_dot_dot)
    external_torque = np.matmul(J.T, wrench_external)
    total_torque = inertia_torque + coriolis_torque + gravity_torque + external_torque
    
    print'Joint Space'
    print'q:\t\n', np.around(q, 3).T
    print'q (inverse kinematics):\n', np.around(q_hat, 3).T
    print'q_dot:\t\n', np.around(q_dot, 3).T
    print'q_dot (inverse kinematics):\n', np.around(q_dot_hat, 3).T
    print'q_dot_dot:\t\n', np.around(q_dot_dot, 3).T
    
    print'\nTask Space'
    print'p (forward kinematics):\t\n', np.around(p, 3).T
    print'p_dot (forward kinematics):\t\n', np.around(p_dot, 3).T
    # currently no way to calculate p_dot_dot
    print'external wrench:\n', np.around(wrench_external, 3).T
    
    print'\nDynamics'
    print'gravity torques:\n', np.around(gravity_torque, 3).T
    print'coriolis torques:\n', np.around(coriolis_torque, 3).T
    print'inertia torques:\n', np.around(inertia_torque, 3).T
    print'external torques:\n', np.around(external_torque, 3).T
    print'feed forward torques:\n', np.around(total_torque, 3).T

if __name__ == "__main__":
    main()
