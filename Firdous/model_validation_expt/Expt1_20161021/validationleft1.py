#!/usr/bin/env python
from __future__ import division, print_function

import math
import datetime
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
        return (limb_joints)
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
    return 0



# initial start postion for x , y ,z
'''
left_point_x=0.811822240501084
left_point_y=0.3329054234877885 
left_point_z=0.0338670648932424
'''
left_point_x=0.657
left_point_y=0.103
left_point_z=0.038
left_yaw   = 1.57
left_pitch =  0
left_roll  =  0
'''
limb_left = baxter_interface.Limb('left')
angles_left             = limb_left.joint_angles()
efforts_left            = limb_left.joint_efforts()
f=open('validation_1.txt','a+')
angles=str(angles_left.keys())
torque=str(angles_left.values())
'''



for inter in range(31):
    left_point_x = left_point_x + 0.005
    left_q=tf.quaternion_from_euler(left_yaw,left_pitch,left_roll,'sxyz')
    print ('the rotation xyz is ')
    print(left_q)
    
    left_x_rotation=left_q[0]
    left_y_rotation=left_q[1]
    left_z_rotation=left_q[2]
    left_w_rotation=left_q[3]
    get_joint_angles('left',left_point_x,left_point_y,left_point_z,left_x_rotation,left_y_rotation,left_z_rotation,left_w_rotation)
    

    print ("+++++++++++++++++++++++++++++++++++++")
    print (datetime.datetime.now())
    limb_left = baxter_interface.Limb('left')
    
    angles_left             = limb_left.joint_angles()
    print (angles_left)
    velocities_left         = limb_left.joint_velocities()
    print (angles_left)
    efforts_left            = limb_left.joint_efforts()
    print (efforts_left)
    
    endpointpose_left       = limb_left.endpoint_pose()
    print (endpointpose_left)


    print (angles_left.keys())
    print (angles_left.values())
    print (endpointpose_left.keys())
    print (endpointpose_left.values())
    print ("+++++++++++++++++++++++++++++++++++++")
    
    f=open('validation_line_test10.txt','a+')

    '''    
    joint=str(angles_left.keys()[0]+ ' '+ angles_left.keys()[1]+ ' '+ angles_left.keys()[2]+ ' '+ angles_left.keys()[3]+ ' '+ angles_left.keys()[4]+ ' '+ angles_left.keys()[5]+ ' '+ angles_left.keys()[6])
    print ('xxxxxxxxxxxxxxxxxxxxxxxxxxxxx')
    yyy=[]
    yyy.append(angles_left.keys())
    xxx=[]
    xxx.append(angles_left.values())
    print (yyy)
    print (xxx)
    '''
    timestamp     = str(datetime.datetime.now())
    jointnamel    = str(angles_left.keys())
    endposenamel  = str(endpointpose_left.keys())
    anglel        = str(angles_left.values())
    velocityl     = str(velocities_left.values())
    torquel       = str(efforts_left.values())
    endposel      = str(endpointpose_left.values())
    '''
    f.write("Timestamp" +'  ')
    f.write(jointnamel + '  ')
    f.write(jointnamel + '  ')
    f.write(jointnamel + '  ')
    f.write(endposenamel + '\n')
    '''   
    f.write(timestamp +'  ')
    f.write(anglel + '  ')
    f.write(velocityl + '  ')
    f.write(torquel + '  ')
    f.write(endposel + '\n')
    '''
    f.write(str(datetime.datetime.now()) +'\n')
    f.write(str(angles_left) +'\n')
    f.write(str(efforts_left)+'\n')
    '''


#    print(angles_left)            
#    print(velocities_left)        
#    print(efforts_left)



