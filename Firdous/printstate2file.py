#!/usr/bin/env python
from __future__ import division, print_function

import math
import datetime
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





def print_states('filename'):

    limb_left = baxter_interface.Limb('left')
    angles_left             = limb_left.joint_angles()
    velocities_left         = limb_left.joint_velocities()
    efforts_left            = limb_left.joint_efforts()
    endpointpose_left       = limb_left.endpoint_pose()

    limb_right = baxter_interface.Limb('right')
    angles_right             = limb_right.joint_angles()
    velocities_right         = limb_right.joint_velocities()
    efforts_right            = limb_right.joint_efforts()
    endpointpose_right       = limb_right.endpoint_pose()
    
    timestamp     = str(datetime.datetime.now())
    anglel        = str(angles_left.values())
    velocityl     = str(velocities_left.values())
    torquel       = str(efforts_left.values())
    endposel      = str(endpointpose_left.values())

    angler        = str(angles_right.values())
    velocityr     = str(velocities_right.values())
    torquer       = str(efforts_right.values())
    endposer      = str(endpointpose_right.values())

    f.write(timestamp +'  ')
    f.write(anglel + '  ')
    f.write(velocityl + '  ')
    f.write(torquel + '  ')
    f.write(endposel + '  ')

    f.write(angler + '  ')
    f.write(velocityr + '  ')
    f.write(torquer + '  ')
    f.write(endposer + '\n')

