#!/usr/bin/env python
import time

import math
import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid

class GetMap():
    def __init__(self):
        #Approximate range of the initial map
        self.xmin=-8
        self.xmax=11
        self.ymin=-3
        self.ymax=6
        self.remakeMap()

    def remakeMap(self):
        self.OGmap = rospy.wait_for_message("map",OccupancyGrid,timeout = None)
        self.width = self.OGmap.info.width
        self.height = self.OGmap.info.height
        self.resolution = self.OGmap.info.resolution
        self.originX= self.OGmap.info.origin.position.x
        self.originY= self.OGmap.info.origin.position.y
        mapdata = np.array(self.OGmap.data,dtype=np.int8)
        #Transform the map into a matrix
        self.map = mapdata.reshape((self.height,self.width))
        #Turn obstacles from 100 to 1
        self.map[self.map == 100] = 1
        # self.map = self.map[:,::-1]

    def WorldtoMap(self,x,y):
        mx = (int)((x-self.originX)/self.resolution)
        my = (int)((y-self.originY)/self.resolution)
        #Map coordinates arerepresented as (height,width)
        return my,mx

    # Check if the  position nearby is located on an obstacle and do not accept it if it is
    def checkPos(self,x,y):
        Pos_ok=True

        if self.map[self.WorldtoMap(x,y)] !=0:
            Pos_ok=False
        if self.map[self.WorldtoMap(x+0.5,y)] !=0:
            Pos_ok=False
        if self.map[self.WorldtoMap(x-0.5,y)] !=0:
            Pos_ok=False
        if self.map[self.WorldtoMap(x,y+0.5)] !=0:
            Pos_ok=False
        if self.map[self.WorldtoMap(x,y-0.5)] !=0:
            Pos_ok=False
        # rospy.loginfo("{}".format(str(self.WorldtoMap(x,y))))
        return Pos_ok


    #Compared to yaml files, the image is flipped about the x-axis
    def printMap(self):
        plt.matshow(self.map, cmap=plt.cm.gray)
        plt.show()


# a= GetMap()
# # for x in np.arange(0,0.5,0.05):
# #     for y in np.arange(0,0.5,0.05):
# #         a.map[a.WorldtoMap(x,y)]=1

# # for x in np.arange(3,3.5,0.05):
# #     for y in np.arange(-5,-4.5,0.05):
# #         a.map[a.WorldtoMap(x,y)]=1
# # for x in np.arange(-8,11.5,0.05):
# #     for y in np.arange(1,1.5,0.05):
# #         a.map[a.WorldtoMap(x,y)]=1
# # for x in np.arange(0,0.5,0.05):
# #     for y in np.arange(-4,6,0.05):
# #         a.map[a.WorldtoMap(x,y)]=1
# for x in np.arange(1,1.5,0.05):
#     for y in np.arange(0,0.1,0.05):
#         print (a.map[a.WorldtoMap(x,y)])
# a.printMap()

