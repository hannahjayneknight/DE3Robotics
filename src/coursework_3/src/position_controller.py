import argparse
import struct
import copy
import os
import sys
import time
import rospy
import numpy as np
import tf


#import baxter_interface

from baxter_core_msgs.srv import (SolvePositionIK, SolvePositionIKRequest)
from gazebo_msgs.srv import (SpawnModel, DeleteModel)
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)
from std_msgs.msg import (Header, Empty)


################################################
#### AUXILIARY FUNCTIONS ########################
def load_gazebo_models():
    """ load all the gazebo models used for this section of the coursework """
    # Poses to spawn the models
    poses = [Pose(position=Point(x=0.75, y=0.45, z=0.0)),
             Pose(position=Point(x=0.75, y=-0.45, z=0.0)),
             Pose(position=Point(x=0.75, y=0.5, z=0.9)),
             Pose(position=Point(x=0.75, y=-0.5, z=0.9)),
             Pose(position=Point(x=0.75, y=0.0, z=0.9)),
             Pose(position=Point(x=0.75, y=0.5, z=0.88))]
    # Files locations of each model
    files = ["cafe_table", "cafe_table",
             "pick_plate", "place_plate", 
             "middle_plate", "Brick"]
    # Names of each model for Gazebo
    names = ["cafe_table_1", "cafe_table_2",
             "pick_plate", "place_plate", 
             "middle_plate", "brick"]
    # Reference frame for spawning the models
    reference_frame = "world" 

    # For each pose
    for pose, file, name in zip(poses,files,names):
        # Load xml from SDF
        model_xml = ''
        with open ("models/"+file+"/model.sdf", "r") as model_file:
            model_xml=model_file.read().replace('\n', '')
        # Spawn model
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        try:
            spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            resp_sdf = spawn_sdf(name, model_xml, "/",
                                pose, reference_frame)
        except:
            pass 


def delete_gazebo_models():
    """ Delete all the gazebo models used for this section of the coursework. """

    models = ["cafe_table_1", "cafe_table_2", "pick_plate", "place_plate", 
                "middle_plate", "brick"]
    
    # For each model, delete it from the world space
    for model in models:
        try:
            delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            resp_delete = delete_model(model)
        except:
            pass


class BaxterArm(object):
    """ Class to operate the Baxter robot arm """

    def __init__(self, limb, verbose=True):
        # Initialise the arm
        self._limb_name = limb                                                          # name (string) of limb
        self._limb = baxter_interface.Limb(limb)                                        # physical model of limb
        self._gripper = baxter_interface.Gripper(limb)                                  # physical model of gripper
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)                           # call ROS service for solving the IK positions
        rospy.wait_for_service(ns, 5.0)                                                 # wait for rospy service to be active
        
        # Verbose flag (for greater detail when debugging)
        self._verbose = verbose 
        
        # Enable the robot
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)         # read in the robot
        self._init_state = self._rs.state().enabled                                     # initiate the robot limbs
        print("Enabling robot... ")
        self._rs.enable()                                                               # enable movement of the limbs
        self._limb.set_joint_position_speed(0.1)                                        # set the speed of the joing positions

    def ik_request(self, pose):
        """ Uses Baxter's internal inverse kinematics solver to calculate inverse position kinematics """
        # Set up the pose message sent to the solver
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)   # send the pose to the solver, get response (resp)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
            
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
                         
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print '[INFO] IK joint solution found!!!'
                print '========================= Joint Space Target State ===================='
                # print 'q_t:\n', q_hat
                for joint in limb_joints:
                    print joint+': ', np.round(limb_joints[joint],4)
                # print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
            return limb_joints
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False

    def move_to_joint_position(self, joint_angles):
        """ move the robot arm to given joint angles """
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        """ Open the gripper """
        print '[ACTION] Open '+self._limb_name+' gripper...'
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        """ Close the gripper """
        print '[ACTION] Close '+self._limb_name+' gripper...'
        self._gripper.close()
        rospy.sleep(1.0)

    def servo_to_pose(self, xyz, rpy):
        """ High level function to move to end effector pose defined by position xyz and orientation rpy """
    
        print '========================= Task Space Target State =========================='
        print 'position:'
        print 'x: ', np.around(xyz[0], 4)
        print 'y: ', np.around(xyz[1], 4)
        print 'z: ', np.around(xyz[2], 4)
        print 'orientation:'
        print 'roll:', np.around(rpy[0], 4)
        print 'pitch:', np.around(rpy[1], 4)
        print 'yaw:', np.around(rpy[2], 4)
        roll, pitch, yaw = rpy
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw) # calculating the quaternion from roll pitch and yaw
        print 'quaternion:' 
        print 'x: ', np.around(quaternion[0], 4)
        print 'y: ', np.around(quaternion[1], 4)
        print 'z: ', np.around(quaternion[2], 4)
        print 'w: ', np.around(quaternion[3], 4)
        print("------------------")
        
        # setting up for the inverse kinematics solver
        ik_pose = Pose() # initializing ROS message to send desired position and orientation of the end-effector
        ik_pose.position.x = xyz[0] # setting the position based on whats passed into the function
        ik_pose.position.y = xyz[1]
        ik_pose.position.z = xyz[2] 
        ik_pose.orientation.x = quaternion[0] # setting the orientation based on whats passed into the function
        ik_pose.orientation.y = quaternion[1]
        ik_pose.orientation.z = quaternion[2]
        ik_pose.orientation.w = quaternion[3]

        print '[ACTION] Finding inverse kinematic solution...'
        joint_angles = self.ik_request(ik_pose) # inverse kinematics to find the joint angles for the end-effector to be in the desired position and orientation
        print '[ACTION] Move the '+self._limb_name+' arm to the target pose!!!'
        self.move_to_joint_position(joint_angles) # moving to the desired position that was found from ik
        print '[INFO] Target pose is achieved!!!'
                        

def main(task):
    print 'Initialitation...'
    try:
        # tuck DENIRO's arms. Run three times in case it fails on one attempt
        cmd = 'rosrun baxter_tools tuck_arms.py -u'
        os.system (cmd)
        cmd = 'rosrun baxter_tools tuck_arms.py -u'
        os.system(cmd)
        cmd = 'rosrun baxter_tools tuck_arms.py -u'
        os.system(cmd)
    except:
        print 'Initialitation failed...'
        print 'Terminating...'
        sys.exit()
    print 'Starting...'
     # Delete Gazebo Models via Spawning Services
    delete_gazebo_models()
    # Load Gazebo Models via Spawning Services
    load_gazebo_models()

    # Initialise baxter arms
    left_arm = BaxterArm('left')
    right_arm = BaxterArm('right')

    # Example commands to open and close gripper
    left_arm.gripper_close()
    left_arm.gripper_open()
    right_arm.gripper_close()
    right_arm.gripper_open()

    # ========= ========= TASK B part i ========= =========
    # Find the correct orientation for the starting pose
    # Hint: Use the knowledge of 3D transformation 
    #       from lectures last term
    # ========= ========= ============= ========= =========
    # Section 2.2.1 in report
    # Example command to move the arms to a target pose with position control
    # Go to the starting Pose for left arm
    left_xyz = [0.5, 0.5, 0.25]
    left_rpy = [0.01, np.pi, 0.01]    # Replace the roll, pitch, and yaw value with the correct value (in radians)
    left_arm.servo_to_pose(left_xyz, left_rpy)
    
    # Go to the starting Pose for right arm
    right_xyz = [0.5, -0.5, 0.25]
    right_rpy = [0.01, np.pi, 0.01]   # Replace the roll, pitch, and yaw value with the correct value (in radians)
    right_arm.servo_to_pose(right_xyz, right_rpy)

    if task == 'initPose':
        return

    elif task == 'pnp1':
        # ========= ========= TASK B part ii ========= =========
        # Find the way points trajectory 
        # Trajectory : a sequence of robot EE pose (xyz, rpy), 
        #              including the gripper state
        # Task: Handling over brick task
        # Execute the trajectory
        # ========= ========= ============== ========= =========
        # Section 2.2.2 in report
        
        # Define the z value for the gripper above the brick
        z_brick = 0.861095+0.192/2-0.93-0.1127/2                # brick_z_coord + brick_height/2 - base_z_coord - gripper_height/2
        
        left_waypoints = [[0.75, 0.5, z_brick], [0.75,  0.5, 0], [0.75,  0, 0], [0.75, 0, z_brick], [0.75, 0, 0.25]]
        
        # For the left arm
        # Move above the brick on the blue circle
        left_arm.servo_to_pose(left_waypoints[0], left_rpy)     # forwards and down
        left_arm.gripper_close()                                # grab
        
        # Move across to the red circle
        left_arm.servo_to_pose(left_waypoints[1], left_rpy)     # up
        left_arm.servo_to_pose(left_waypoints[2], left_rpy)     # right
        left_arm.servo_to_pose(left_waypoints[3], left_rpy)     # down
        left_arm.gripper_open()                                 # release
        
        # Move back to initial position
        left_arm.servo_to_pose(left_waypoints[4], left_rpy)     # up
        rospy.sleep(10)
        left_arm.servo_to_pose(left_xyz, left_rpy)              # initial
        
        # For the right arm
        # Define waypoints for the right arm
        right_waypoints = [[0.75, 0, 0.25], [0.75, 0, z_brick], [0.75,  0, 0], [0.75, -0.5, 0], [0.75, -0.5, z_brick], [0.75, -0.5, 0.25]]
        
        # Move above the brick on the red circle
        right_arm.servo_to_pose(right_waypoints[0], right_rpy)  # forwards and left
        right_arm.servo_to_pose(right_waypoints[1], right_rpy)  # down
        right_arm.gripper_close()                               # grab
        
        # Move across to the green circle
        right_arm.servo_to_pose(right_waypoints[2], right_rpy)  # up
        right_arm.servo_to_pose(right_waypoints[3], right_rpy)  # right
        right_arm.servo_to_pose(right_waypoints[4], right_rpy)  # down
        right_arm.gripper_open()                                # release
        
        # Move back to initial position
        right_arm.servo_to_pose(right_waypoints[5], right_rpy)  # up
        right_arm.servo_to_pose(right_xyz, right_rpy)           # initial

        return

    elif task == 'pnp2':
        # ========= ========= TASK B part iii ========= =========
        # Find the way points trajectory 
        # Trajectory : a sequence of robot EE pose (xyz, rpy), 
        #              including the gripper state
        # Task: Handling over brick task without intermediate 
        #       placement
        # Execute the trajectory
        # ========= ========= =============== ========= =========
        # Section 2.2.3 in report
        
        # Define the z value for the gripper above the brick
        z_brick = 0.871095+0.192/2-0.93-0.1127/2                # brick_z_coord + brick_height/2 - base_z_coord - gripper_height/2
        
        left_waypoints = [[0.75,  0.5, 0.25], [0.75, 0.5, z_brick], [0.75,  0.5, 0.12], [0.75,  0.1, 0.12], [0.75,  0.05, 0.12]]
        right_waypoints = [[0.75, -0.5, 0.12], [0.75, -0.05, 0.12], [0.75, 0, 0.12], [0.75, -0.5, z_brick], [0.75, -0.5, 0.25]]
        rotated_pos_x, rotated_neg_x = [np.pi/2, np.pi, 0.01], [-np.pi/2, np.pi, 0.01]
        
        # Move above the brick on the blue circle
        left_arm.servo_to_pose(left_waypoints[0], left_rpy)     # inital
        left_arm.servo_to_pose(left_waypoints[1], left_rpy)     # to brick
        left_arm.gripper_close()                                # grab
        
        # Rotate the left arm
        left_arm.servo_to_pose(left_waypoints[2], left_rpy)         # up
        left_arm.servo_to_pose(left_waypoints[2], rotated_pos_x)    # then turn
        left_arm.servo_to_pose(left_waypoints[3], rotated_pos_x)    # move to middle
        right_arm.servo_to_pose(right_waypoints[0], rotated_neg_x)  # turn
        
        # Hand over the brick to the right arm
        right_arm.servo_to_pose(right_waypoints[1], rotated_neg_x)  # to brick
        right_arm.servo_to_pose(right_waypoints[2], rotated_neg_x)  # to brick
        left_arm.servo_to_pose(left_waypoints[4], rotated_pos_x)    # move left arm closer
        right_arm.gripper_close()                                   # grab
        left_arm.gripper_open()                                     # release
        
        # Return the left arm back to initial position
        left_arm.servo_to_pose(left_waypoints[0], left_rpy)         # carefully slot out of brick
        left_arm.servo_to_pose(left_xyz, left_rpy)                  # left arm back to initial
        
        # Rotate the right arm
        right_arm.servo_to_pose(right_waypoints[0], rotated_neg_x)  # move
        right_arm.servo_to_pose(right_waypoints[0], right_rpy)      # turn
        right_arm.servo_to_pose(right_waypoints[3], right_rpy)      # down
        
        # Place the brick on the green circle and return the right arm back to the original position
        right_arm.gripper_open()                                    # release
        right_arm.servo_to_pose(right_waypoints[4], right_rpy)      # up
        right_arm.servo_to_pose(right_xyz, right_rpy)               # take right arm to initial pos

        return
   

if __name__ == "__main__":
    """Simple pick-handover-place demo"""
    rospy.init_node("position_control")
    
    tasks = ['initPose', 'pnp1', 'pnp2']
    if len(sys.argv) <= 1:
        print 'Please include a task to run from the following options:\n', tasks
    else:
        task = str(sys.argv[1])
        if task in tasks:
            print "Running Position Control -", task
            main(task)
        else:
            print 'Please include a task to run from the following options:\n', tasks
    rospy.sleep(2)
    rospy.signal_shutdown("FINISHED")
