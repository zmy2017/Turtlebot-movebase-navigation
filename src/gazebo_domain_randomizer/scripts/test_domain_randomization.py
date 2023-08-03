#! /usr/bin/python
import rospy
import math
import copy
import traceback
import sys
import gazebo_env
from gazebo_msgs.srv import *
from gazebo_ext_msgs.srv import *

if __name__ == "__main__":
    rospy.init_node("test_scenario")
    env = gazebo_env.GazeboEnv()
    # res = env.get_link_prop("wheel_left_link")
    # # print(res)
    # req = SetLinkPropertiesRequest()
    # req.link_name = "wheel_left_link"
    # req.com = copy.deepcopy(res.com)
    # req.mass = copy.deepcopy(res.mass)
    # # print(req)
    # env.set_link_prop(req)

    #joint
    # res =env.get_joint_prop("wheel_left_joint")
    # print(res)
    # req = SetJointPropertiesRequest()
    # req.joint_name="wheel_left_joint"
    # req.ode_joint_config.damping = [3.14]
    # # print(req)
    # env.set_joint_prop(req)

    #surface
    res = env.get_surface_params("wheel_left_link_collision")
    print(res)
    req=SetSurfaceParamsRequest()
    req.link_collision_name="wheel_left_link_collision"
    req.mu1=5
    req.mu2=10
    env.set_surface_params(req)