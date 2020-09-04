#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt
import rospkg 


import numpy as np

class GenerateMap:

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

        # use the rospack object to get paths
        rospack = rospkg.RosPack()
        package_path=rospack.get_path('race')
        # get the pid to create "unique" filenames
        self.filename=package_path+'/maps/{}_obstacles.txt'.format(self.name)


    def save_points(self):
        self.file = open(self.filename, 'w')
        for i in range(len(self.x)):
            xp = self.x[i] 
            yp = self.y[i]
            self.file.write('%f, %f\n' % (xp,yp))
        self.file.close()
        print("Done")
            

    def execute(self):
        msg = self.srv_proxy().map 

        # map metadata
        origin =[msg.info.origin.position.x,msg.info.origin.position.y]
        res = msg.info.resolution

        map_data= np.asarray(msg.data)
        grid_size=(msg.info.width,msg.info.height)
        map_data = map_data.reshape(grid_size)

        occ_x,occ_y = np.where(map_data==100)
        occ_x_free,occ_y_free = np.where(map_data==0)

        print(len(occ_x))
        for i in range(len(occ_x)):
            x_point = res*occ_x[i] + origin[0]
            y_point = res*occ_y[i] + origin[0]
            self.points.append((x_point,y_point))
            self.x.append(y_point)
            self.y.append(x_point)
        self.save_points()

        for i in range(len(occ_x_free)):
            x_point = res*occ_x_free[i] + origin[0]
            y_point = res*occ_y_free[i] + origin[0]
            self.x_free.append(y_point)
            self.y_free.append(x_point)
            
        
        plt.plot(self.x,self.y,'ro')
        #plt.plot(self.x_free,self.y_free,'bo')
        plt.title('Plot of obstacles in track: {}'.format(self.name))
        plt.ylabel('x')
        plt.xlim([-15,15])
        plt.ylim([-15,15])
        plt.xlabel('y')
        plt.show()


if __name__=="__main__":
    rospy.init_node("getMap")
    gm = GenerateMap()
    gm.execute()