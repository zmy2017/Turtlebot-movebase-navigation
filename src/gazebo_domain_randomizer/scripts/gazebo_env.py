# -*- coding: UTF-8 -*-
#!/usr/bin/env python
import math
import os
import random
import subprocess
import time
from os import path

import numpy as np
import rospy
from gazebo_msgs.msg import ModelState
from quaternions import Quaternion
from std_srvs.srv import Empty
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from gazebo_msgs.srv import *
from gazebo_ext_msgs.srv import *
import get_map
import utils

MODEL_NAME = "mobile_base"
TIME_RATE =10


class GazeboEnv:
    """Superclass for all Gazebo environments."""

    def __init__(self) :

        self.real_world =False

        # The publisher needs to be declared in advance
        # Avoid the first topics are not posted 
        self.unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self.pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self.reset_proxy = rospy.ServiceProxy("/gazebo/reset_world", Empty)
        self.get_state_service = rospy.ServiceProxy("/gazebo/get_model_state",GetModelState)
        self.set_model_state = rospy.Publisher(
        "gazebo/set_model_state", ModelState, queue_size=10)        
        self.amcl_pose = rospy.Publisher(
            "initialpose",PoseWithCovarianceStamped,queue_size=10
        )
        self.goal_client=actionlib.SimpleActionClient('move_base',MoveBaseAction)

        self.get_link_prop = rospy.ServiceProxy("/gazebo/get_link_properties",GetLinkProperties)
        self.set_link_prop = rospy.ServiceProxy("/gazebo/set_link_properties",SetLinkProperties)
        self.get_joint_prop = rospy.ServiceProxy("/gazebo/get_joint_properties",GetJointProperties)
        self.set_joint_prop = rospy.ServiceProxy("/gazebo/set_joint_properties",SetJointProperties)
        self.set_surface_params = rospy.ServiceProxy( '/gazebo/set_surface_params', SetSurfaceParams)
        self.get_surface_params = rospy.ServiceProxy( '/gazebo/get_surface_params', GetSurfaceParams)


        # Get map information
        self.get_map = get_map.GetMap()

        #tried opening Gazebo by subprocess,but no better than open Gazebo in advance 
        # port = "11311"
        # subprocess.Popen(["roscore", "-p", port])
        # print("Roscore launched!")
        # subprocess.Popen(["killall","gzserver"])
        # subprocess.Popen(["killall","gzclient"])
        # print("Clean server!")
        # #launch the simlation with given launchfile
        # rospy.init_node("GazeboEnv",anonymous=True)
        # if launchfile.startswith("/"):
        #     fullpath = launchfile
        # else:
        #     fullpath =os.path.join(os.path.dirname(__file__),"os.path.pardir")


    def getModelState(self):
        self.get_state_service.wait_for_service()
        model_req = GetModelStateRequest()
        model_req.model_name = MODEL_NAME
        try:
            res = self.get_state_service(model_req)
            if not res.success:
                rospy.logwarn(res.status_message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        return res 





    def goal_callback(self,event):
        model_state =self.getModelState()
        x = model_state.pose.position.x
        y = model_state.pose.position.y
        z = model_state.pose.position.z
        qx = model_state.pose.orientation.x
        qy = model_state.pose.orientation.y
        qz = model_state.pose.orientation.z
        qw = model_state.pose.orientation.w
        stamp = model_state.header.stamp.to_sec()
        if self.real_world:
            fullpath =os.path.join(os.path.dirname(__file__),"real_traj.txt")
        else:
            fullpath =os.path.join(os.path.dirname(__file__),"sim_traj.txt")
        with open(fullpath,'a') as file:
            file.write("{stamp} {x} {y} {z} {qx} {qy} {qz} {qw}\n".format(stamp=stamp,x=x,y=y,z=z,qx=qx,qy=qy,qz=qz,qw=qw))
        

    def goalMovebase(self,x,y,z,theta,real_world = False):
        # send move_base goal
        # self.goal_client=actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.real_world =real_world
        self.goal_client.wait_for_server()
        if self.real_world:
            fullpath =os.path.join(os.path.dirname(__file__),"real_traj.txt")
        else:
            fullpath =os.path.join(os.path.dirname(__file__),"sim_traj.txt")
        # Empty the contents of the file
        with open(fullpath, 'w') as file:
            pass
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x =x
        goal.target_pose.pose.position.y =y
        goal.target_pose.pose.position.z = z
        quaternion = Quaternion.from_euler([0.0, 0.0, theta])
        goal.target_pose.pose.orientation.x = quaternion.x
        goal.target_pose.pose.orientation.y = quaternion.y
        goal.target_pose.pose.orientation.z = quaternion.z
        goal.target_pose.pose.orientation.w = quaternion.w
        rospy.loginfo("Target pose is located at {} ".format(str((x,y,z,theta))))
        self.goal_client.send_goal(goal)
        timer = rospy.Timer(rospy.Duration(0.1), self.goal_callback)
        wait = self.goal_client.wait_for_result()
        timer.shutdown()
        if not wait :
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            rospy.loginfo("Done")
            return self.goal_client.get_result()
    
    


    # Generate random starting point or target point
    # Special case where the initial map is rectangular
    # The coordinates of the four vertices are estimated
    # After modifying the map, this method should be modified ！！！    
    def resetPose(self):
        x=np.random.uniform(self.get_map.xmin, self.get_map.xmax)
        y=np.random.uniform(self.get_map.ymin, self.get_map.ymax)
        flag = self.get_map.checkPos(x,y)
        while flag is not True:
            rospy.loginfo("Pose {}is invalid,generate pose again ".format(str([x,y])))
            x=np.random.uniform(self.get_map.xmin, self.get_map.xmax)
            y=np.random.uniform(self.get_map.ymin, self.get_map.ymax)
            flag = self.get_map.checkPos(x,y)
        theta = np.random.uniform(-np.pi, np.pi)
        # Return x,y,z,theta as z in the map is always 0 
        return x,y,0.0,theta

    # Set model state only for turtlebot
    def setModelState(self,x,y,z,theta):
        # self.set_model_state = rospy.Publisher(
        # "gazebo/set_model_state", ModelState, queue_size=10)
        model_state = ModelState()
        model_state.model_name = "mobile_base"
        model_state.pose.position.x =x
        model_state.pose.position.y =y
        model_state.pose.position.z =z

        quaternion = Quaternion.from_euler([0.0, 0.0, theta])
        model_state.pose.orientation.x = quaternion.x
        model_state.pose.orientation.y = quaternion.y
        model_state.pose.orientation.z = quaternion.z
        model_state.pose.orientation.w = quaternion.w
        try:
            self.set_model_state.publish(model_state)
        except rospy.ServiceException as e:
            print("/gazebo/set_model_state topic publish failed")

        
    def resetAmcl(self,x,y,z,theta):
        # self.amcl_pose = rospy.Publisher(
        #     "initialpose",PoseWithCovarianceStamped,queue_size=10
        # )
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp =rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x =x
        pose_msg.pose.pose.position.y =y
        pose_msg.pose.pose.position.z =z

        quaternion = Quaternion.from_euler([0.0, 0.0, theta])
        pose_msg.pose.pose.orientation.x = quaternion.x
        pose_msg.pose.pose.orientation.y = quaternion.y
        pose_msg.pose.pose.orientation.z = quaternion.z
        pose_msg.pose.pose.orientation.w = quaternion.w
        try:
            self.amcl_pose.publish(pose_msg)
        except rospy.ServiceException as e:
            print("/amcl_pose topic publish failed")
        




        


    def reset(self,x,y,z,theta):

        #Reset the state of the environment and returns an initial observation
        rospy.wait_for_service("/gazebo/reset_world")
        try:
            self.reset_proxy()
        except rospy.ServiceException as e:
            print("/gazebo/reset_simulation service call failed")
        #set turtlebot start pose
        # x,y,z,theta = self.resetPose()
        rospy.loginfo("Start pose is located at {} ".format(str((x,y,z,theta))))
        self.setModelState(x,y,z,theta)
        self.resetAmcl(x,y,z,theta)
    



    #Link Properties

    def getLinkProperties(self,link_name):
        try:
            res = self.get_link_prop(GetLinkPropertiesRequest(link_name=link_name))
            if not res.success:
                rospy.logwarn(res.status_message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        return res        


    def setLinkProperties(self,req):
        try:
            res = self.set_link_prop(req)
            if not res.success:
                rospy.logwarn(res.status_message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    #Joint Properties
    #Attention:A great difference of data structure between getJointProprties and setJointProprties
    #View specify structure by calling corresponding service
    def getJointProperties(self,joint_name):
        try:
            res = self.get_joint_prop(GetJointPropertiesRequest(joint_name=joint_name))
            if not res.success:
                rospy.logwarn(res.status_message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        return res        


    def setJointProperties(self,req):
        try:
            res = self.set_joint_prop(req)
            if not res.success:
                rospy.logwarn(res.status_message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    #Surface Properties :customized service
    def getSurfaceParams(self,link_collision_name):
        try:
            res = self.get_surface_params(GetSurfaceParamsRequest(link_collision_name=link_collision_name))
            if not res.success:
                rospy.logwarn(res.status_message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        return res        


    def setSurfaceParams(self,req):
        try:
            res = self.set_surface_params(req)
            if not res.success:
                rospy.logwarn(res.status_message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
