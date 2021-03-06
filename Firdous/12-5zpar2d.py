#!/usr/bin/env python
from __future__ import division, print_function

import math
import tf
import numpy
import time
import doctest
import random  # used in doctests
import argparse
import sys
import rospy
import baxter_interface
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
__version__ = '2015.07.18'
__docformat__ = 'restructuredtext en'
__all__ = ()
'''
limbs = ('left', 'right')
arms = {
            'left': baxter_interface.Limb('left'),
            'right': baxter_interface.Limb('right'),
            }

'''
# create an instance of baxter_interface's Limb class

def get_joint_angles(limb,Px,Py,Pz,Qx,Qy,Qz,Qw):

    rospy.init_node("rsdk_ik_service_client")
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
    'left': PoseStamped(
        header=hdr,
        pose=Pose(
            position=Point(
                x = Px,    #original val:  0.657579481614,
                y = Py,    #original val:  0.851981417433,
                z = Pz,    #original val:  0.0388352386502,
            ),
            orientation=Quaternion(
                x = Qx, #original val: -0.366894936773,
                y = Qy, #original val:  0.885980397775,
                z = Qz, #original val:  0.108155782462,
                w = Qw, #original val:  0.262162481772,
            ),
        ),
    ),
    'right': PoseStamped(
        header=hdr,
        pose=Pose(
            position=Point(
                x=Px, #original val:  0.656982770038,
                y=Py, #original val: -0.852598021641,
                z=Pz, #original val:  0.0388609422173,
            ),
            orientation=Quaternion(
                x=Qx, #original val:  0.367048116303,
                y=Qy, #original val:  0.885911751787,
                z=Qz, #original val: -0.108908281936,
                w=Qw, #original val:  0.261868353356,
            ),
        ),
    ),
    }

    #print (poses)
    ikreq.pose_stamp.append(poses[limb])
    
    
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1
    if (resp.isValid[0]):
        print("SUCCESS - Valid Joint Solution Found:")
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        #print limb_joints
        if limb == 'left':
            limb_left = baxter_interface.Limb('left')
            limb_left.move_to_joint_positions(limb_joints)
        if limb == 'right':
            limb_right = baxter_interface.Limb('right')
            limb_right.move_to_joint_positions(limb_joints)
        return (limb_joints)
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
    return 0



def parallel_move():
        # initial start postion for x , y ,z
    left_point_x=0.657
    left_point_y=0.0685
    left_point_z=0.038

    right_point_x=0.657
    right_point_y=-0.170
    right_point_z=0.038

    '''
    left_yaw=0
    left_pitch=0
    left_roll=0.5*math.pi
    right_yaw=0
    right_pitch=0
    right_roll=-0.5*math.pi
    '''

    left_yaw=0.5*math.pi
    left_pitch=0
    left_roll=0
    right_yaw=-0.5*math.pi
    right_pitch=0
    right_roll=0
    change_angle=0

    initial_x=left_point_x

    for inter_z in range(2): 
        left_point_x=initial_x
        right_point_x=initial_x
        left_point_z=left_point_z-0.01*10
        right_point_z=right_point_z-0.01*10

        for inter in range(10):
            left_point_x=left_point_x+0.01
            right_point_x=right_point_x+0.01

            #left_pitch=left_pitch+0.1

            left_q=tf.quaternion_from_euler(left_yaw,left_pitch,left_roll,'sxyz')
            right_q=tf.quaternion_from_euler(right_yaw,right_pitch,right_roll,'sxyz')
            #print ('the rotation xyz is ')
            #print(left_q)

            left_x_rotation=left_q[0]
            left_y_rotation=left_q[1]
            left_z_rotation=left_q[2]
            left_w_rotation=left_q[3]

            right_x_rotation=right_q[0]
            right_y_rotation=right_q[1]
            right_z_rotation=right_q[2]
            right_w_rotation=right_q[3]

            get_joint_angles('left',left_point_x,left_point_y,left_point_z,left_x_rotation,left_y_rotation,left_z_rotation,left_w_rotation)
            print ("left_done: "+str(inter))
            get_joint_angles('right',right_point_x,right_point_y,right_point_z,right_x_rotation,right_y_rotation,right_z_rotation,right_w_rotation)
            print ("right_done: "+str(inter))
            pause_time=5
            print ("All_done: "+str(inter)+' pause for '+str(pause_time))
            time.sleep(pause_time)



    return 0


def angular_move():


    '''
    left_yaw=-0.5*math.pi
    left_pitch=0
    left_roll=0
    right_yaw=0.5*math.pi
    right_pitch=0
    right_roll=0
    change_angle=0
    '''

# another way for intial angle
    left_yaw=0.5*math.pi
    left_pitch=0
    left_roll=0
    right_yaw=-0.5*math.pi
    right_pitch=0
    right_roll=0
    change_angle=0

    #
    intial_left_yaw=left_yaw
    intial_right_yaw=right_yaw
     #phatom width 
    distance_a=0.27
    #distance from cam to edge of phatom
    distance_b=0.005
    #distance from laser to edge of phatom
    distance_c=0.081
        #position of left gripper
    distance_d=0.090
    distance_e=0.040
    distance_h=0.1
    distance_g=0.094
    distance_l=0.150
    distance_i=0.5
    R2=0.5*distance_l+distance_c+distance_a+distance_b+0.5*distance_h
    
#initial position5
    left_point_x=distance_i+0.5*distance_d
    left_point_y= distance_b+0.5*distance_a+0.5*distance_h
    left_point_z=0.01
    right_point_x=left_point_x
    right_point_z=left_point_z
    right_point_y=-(distance_c+0.5*distance_a+0.5*distance_l)




    for inter in range(5):





        left_q=tf.quaternion_from_euler(left_yaw,left_pitch,left_roll,'sxyz')
        right_q=tf.quaternion_from_euler(right_yaw,right_pitch,right_roll,'sxyz')

        left_x_rotation=left_q[0]
        left_y_rotation=left_q[1]
        left_z_rotation=left_q[2]
        left_w_rotation=left_q[3]
        get_joint_angles('left',left_point_x,left_point_y,left_point_z,left_x_rotation,left_y_rotation,left_z_rotation,left_w_rotation)
        print ("left_done: "+str(inter))
        right_x_rotation=right_q[0]
        right_y_rotation=right_q[1]
        right_z_rotation=right_q[2]
        right_w_rotation=right_q[3]
        get_joint_angles('right',right_point_x,right_point_y,right_point_z,right_x_rotation,right_y_rotation,right_z_rotation,right_w_rotation)
        print ("right_done: "+str(inter))
        pause_time=5
        print ("All_done: "+str(inter)+' pause for '+str(pause_time))
        time.sleep(pause_time)

        change_angle+=math.radians(5)
        left_yaw=change_angle+intial_left_yaw
        right_yaw=change_angle+intial_right_yaw




        left_point_x=left_point_x-R2*math.sin(change_angle)
        left_point_y=R2*math.cos(change_angle)-(distance_c+0.5*distance_a+0.5*distance_l)



    return 0

'''

def angular_move():


    left_yaw=-0.5*math.pi
    left_pitch=0
    left_roll=0
    right_yaw=0.5*math.pi
    right_pitch=0
    right_roll=0


    #another way for gripper





    for inter in range(10):


        left_q=tf.quaternion_from_euler(left_yaw,left_pitch,left_roll,'sxyz')
        right_q=tf.quaternion_from_euler(right_yaw,right_pitch,right_roll,'sxyz')

        left_x_rotation=left_q[0]
        left_y_rotation=left_q[1]
        left_z_rotation=left_q[2]
        left_w_rotation=left_q[3]
        get_joint_angles('left',0.581,0.0685,0.10,left_x_rotation,left_y_rotation,left_z_rotation,left_w_rotation)
         #get_joint_angles('left',left_point_x,left_point_y,left_point_z,left_x_rotation,left_y_rotation,left_z_rotation,left_w_rotation)

        right_x_rotation=right_q[0]
        right_y_rotation=right_q[1]
        right_z_rotation=right_q[2]
        right_w_rotation=right_q[3]
        get_joint_angles('right',0.581,-0.1695,0.10,right_x_rotation,right_y_rotation,right_z_rotation,right_w_rotation)
        left_yaw+=math.radians(1)
        right_yaw+=math.radians(1)
    return 0
'''

if __name__ == "__main__":
    #parallel_move()
    angular_move()


