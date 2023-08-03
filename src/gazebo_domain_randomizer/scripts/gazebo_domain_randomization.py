#!/usr/bin/env python
import math
import os
import random
import time
from os import path
import copy
import numpy as np
import rospy

from gazebo_msgs.srv import *
from gazebo_ext_msgs.srv import *

class DR_Turtlebot:
    def __init__(self,env,dr_option =  "friction_mass_dr",real_world = False):
        # use GazeboEnv function
        self.env =env

        self.dr_option = dr_option
        self.real_world = real_world
        self.config_dr()


    def config_dr(self):
        
        self.real_dr_values = {
            "wheel_left_link_collision_mu":3.4,
            "wheel_right_link_collision_mu":4.5,
            "base_footprint_mass":10
        }
        self.sim_dr_values_initial = {
            "wheel_left_link_collision_mu":10.0,
            "wheel_right_link_collision_mu":0.1,
            "base_footprint_mass":30
        }
        self.sim_dr_values =copy.deepcopy(self.sim_dr_values_initial)
        # all parameters domain randomization(more parameters in the future)
        if self.dr_option == "all_dr":
            self.dr_list = list(self.real_dr_values.keys())
        if self.dr_option == "friction_mass_dr":
            self.dr_list = ["wheel_left_link_collision_mu","wheel_right_link_collision_mu","base_footprint_mass"]
    
    def applySurfaceDR(self,name,surface_dr_list):
        env = self.env
        res = env.getSurfaceParams(name)
        req=SetSurfaceParamsRequest()
        #Copy common parameters
        req.link_collision_name= name
        req.elastic_modulus =res.elastic_modulus
        req.mu1= res.mu1
        req.mu2= res.mu2
        req.mu_torsion=res.mu_torsion
        req.patch_radius =res.patch_radius
        req.poisson_ratio= res.poisson_ratio

        if name+'_mu' in surface_dr_list:
            if self.real_world:
                value =self.real_dr_values[name+'_mu']
            else:
                value =self.sim_dr_values[name+'_mu']
            req.mu1= value
            req.mu2 =value
        env.setSurfaceParams(req)

    def applyLinkPropertiesDR(self,name,link_properties_dr_list):
        env =self.env
        res = env.getLinkProperties(name)
        req = SetLinkPropertiesRequest()
        req.link_name = name
        req.com = res.com
        req.gravity_mode = res.gravity_mode
        req.mass = res.mass
        req.ixx = res.ixx
        req.ixy = res.ixy 
        req.ixz = res.ixz
        req.iyy = res.iyy
        req.iyz = res.iyz
        req.izz = res.izz
        if name+'_mass' in link_properties_dr_list:
            if self.real_world:
                value =self.real_dr_values[name+'_mass']
            else:
                value =self.sim_dr_values[name+'_mass']
            req.mass = value
        env.setLinkProperties(req)

    def applyDR(self):
        #LinkProperties
        wheel_left_link_list = [s for s in self.dr_list if "wheel_left_link" in s and "collision" not in s]
        if wheel_left_link_list:
            self.applyLinkPropertiesDR("wheel_left_link",wheel_left_link_list)
        wheel_right_link_list = [s for s in self.dr_list if "wheel_right_link" in s and "collision" not in s]
        if wheel_right_link_list:
            self.applyLinkPropertiesDR("wheel_right_link",wheel_right_link_list)
        base_footprint_list = [s for s in self.dr_list if "base_footprint" in s and "collision" not in s]
        if base_footprint_list:
            self.applyLinkPropertiesDR("base_footprint",base_footprint_list)
        #SurfaceParams
        wheel_left_link_collision_list = [s for s in self.dr_list if "wheel_left_link_collision" in s]
        if wheel_left_link_collision_list:
            self.applySurfaceDR("wheel_left_link_collision",wheel_left_link_collision_list)
        wheel_right_link_collision_list = [s for s in self.dr_list if "wheel_right_link_collision" in s]
        if wheel_right_link_collision_list:
            self.applySurfaceDR("wheel_right_link_collision",wheel_right_link_collision_list)


