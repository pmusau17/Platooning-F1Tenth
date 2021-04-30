#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
import matplotlib.pyplot as plt
import rospkg 
import matplotlib.pyplot as plt


import numpy as np

class SaveMap:
    """Generates walls of maps:
       Note: -1: Unknown
              0: Free
              100: Occupied """
    def __init__(self,threshold=0.65,name="porto"):
        rospy.wait_for_service('static_map')
        self.srv_proxy = rospy.ServiceProxy('static_map', GetMap)
        self.threshold = threshold
        self.points =[]
        self.x = []
        self.y = []
        self.x_free = []
        self.y_free = []
        self.name = name
        self.offset_x = 0
        self.offset_y = 0

        # use the rospack object to get paths
        rospack = rospkg.RosPack()
        package_path=rospack.get_path('race')
        # get the pid to create "unique" filenames
        self.filename=package_path+'/maps/{}_grid.npy'.format(self.name)

    def execute(self):
        msg = self.srv_proxy().map 

        # map metadata
        origin =[msg.info.origin.position.y+self.offset_x,msg.info.origin.position.x-self.offset_y]
        #origin = [0,0]
        res = msg.info.resolution
        map_data= np.asarray(msg.data)
        grid_size=(msg.info.height,msg.info.width)
        map_data = map_data.reshape(grid_size)
        np.save(self.filename, map_data)
        plt.imshow(map_data)
        plt.title('Plot of Walls and Freespace in track: {}'.format(self.name))
        plt.ylabel('x')
        plt.xlabel('y')
        plt.show()


if __name__=="__main__":
    rospy.init_node("getMap")
    args = rospy.myargv()[1:]
    track_name=args[0:1]
    generate_freespace=args[1:2]
    sm=SaveMap()
    sm.execute()