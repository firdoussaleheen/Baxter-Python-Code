# Import the necessary Python modules

import sys
#import print_function

# rospy - ROS Python API
import rospy

# baxter_interface - Baxter Python API
import baxter_interface

# initialize our ROS node, registering it with the Master
rospy.init_node('Hello_Baxter')

# create an instance of baxter_interface's Limb class
limb_left = baxter_interface.Limb('left')


# get the left limb's current joint angles
angles_left             = limb_left.joint_angles()
velocities_left         = limb_left.joint_velocities()
efforts_left            = limb_left.joint_efforts()
endpointpose_left       = limb_left.endpoint_pose()
endpointvelocity_left   = limb_left.endpoint_velocity()
endpointeffort_left     = limb_left.endpoint_effort()


print angles_left            
print velocities_left        
print efforts_left           
print endpointpose_left      
print endpointvelocity_left  
print endpointeffort_left    

#from __future__ 
#f = open("output.txt","w")

#print("angles_left", file = f)            
#print("velocities_left", file = f)        
#print("efforts_left", file = f)           
#print("endpointpose_left", file = f)      
#print("endpointvelocity_left", file = f)  
#print("endpointeffort_left", file = f)    

#f.close()

quit()
