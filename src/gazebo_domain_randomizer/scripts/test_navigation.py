#! /usr/bin/python
import rospy
import math

import traceback
import sys
import gazebo_env
import gazebo_domain_randomization

if __name__ == "__main__":
  rospy.init_node("test_scenario")
  env = gazebo_env.GazeboEnv()
  sim_env  = gazebo_domain_randomization.DR_Turtlebot(env,real_world = False)
  real_env  = gazebo_domain_randomization.DR_Turtlebot(env,real_world = True)
  
  rospy.loginfo("Initializations done")
  # Sleep is necessaryfor GazeboEnv initialization
  rospy.sleep(3)
  for i in range(0,1):
    #sim env
    sim_env.applyDR()
    #start pose
    start_x,start_y,start_z,start_theta = env.resetPose()
    #target_pose
    target_x,target_y,target_z,target_theta = env.resetPose()
    env.reset(start_x,start_y,start_z,start_theta)
    env.goalMovebase(target_x,target_y,target_z,target_theta,sim_env.real_world)

    #real env
    real_env.applyDR()
    env.reset(start_x,start_y,start_z,start_theta)
    env.goalMovebase(target_x,target_y,target_z,target_theta,real_env.real_world)




  rospy.loginfo("test terminated.")