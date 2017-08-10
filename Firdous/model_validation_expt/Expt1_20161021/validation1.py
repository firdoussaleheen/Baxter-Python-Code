#!/usr/bin/env python
from __future__ import division, print_function

import math
import tf
import numpy
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

    print (poses)
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





# initial start postion for x , y ,z
left_point_x=0.811822240501084,
left_point_y=0.3329054234877885, 
left_point_z=0.0338670648932424,

right_point_x=0.657
right_point_y=-0.103
right_point_z=0.038


left_yaw   = -0.0029
left_pitch =  0.0589
left_roll  =  1.7006
right_yaw  =0
right_pitch= 0
right_roll =1.5789105278117897

for inter in range(3):
    left_point_x=left_point_x+0.01
    left_q=tf.quaternion_from_euler(left_yaw,left_pitch,left_roll,'sxyz')
    right_q=tf.quaternion_from_euler(right_yaw,right_pitch,right_roll,'sxyz')
    print ('the rotation xyz is ')
    print(left_q)
    
    left_x_rotation=left_q[0]
    left_y_rotation=left_q[1]
    left_z_rotation=left_q[2]
    left_w_rotation=left_q[3]
    get_joint_angles('left',left_point_x,left_point_y,left_point_z,left_x_rotation,left_y_rotation,left_z_rotation,left_w_rotation)
    
#    angles_left             = limb_left.joint_angles()
#    velocities_left         = limb_left.joint_velocities()
#    efforts_left            = limb_left.joint_efforts()
#    print(angles_left)            
#    print(velocities_left)        
#    print(efforts_left)


