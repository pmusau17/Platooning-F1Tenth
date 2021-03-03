#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import matplotlib.animation as animation
#need to subscribe to the steering message and angle message
from message_filters import ApproximateTimeSynchronizer, Subscriber
import seaborn as sns 
import numpy as np

class PlotReachability:
    def __init__(self):
        #self.reachability_result=rospy.Subscriber('reachability_result',Float32,self.callback)
        #self.reachability_result=rospy.Subscriber('racecar2/scan',LaserScan,self.laser_callback)

        self.reach_result=Subscriber('reachability_result',Float32)
        self.reach_result2=Subscriber('disparity/angle_msg/reachability_result',Float32)
        self.reach_result3=Subscriber('pure_pursuit/angle_msg/reachability_result',Float32)

        self.results=[]
        self.results2 =[]
        self.results3 =[] 
        self.times=[]

        self.fig,self.axes = plt.subplots(figsize=(25,8))
        #self.ax = self.fig.add_subplot(1, 1, 1)
        self.window=4000
        self.count= 0
        #self.min_distances = []
        #self.min_distance = np.inf

        self.sub = ApproximateTimeSynchronizer([self.reach_result, self.reach_result2,self.reach_result3], queue_size = 20, slop = 0.019,allow_headerless=True)
        self.sub.registerCallback(self.callback)
        #Animation
        # Set up plot to call animate() function periodically
        ani = animation.FuncAnimation(self.fig, self.animate, fargs=(self.results, self.results,self.results,self.times), interval=1000)
        plt.show()


    def callback(self,lec,disp,pure_pursuit):
        res = lec.data
        self.results.append(int(res))
        self.results2.append(int(disp.data))
        self.results3.append(int(pure_pursuit.data))
        #print(self.times[:4],self.results[:4],self.results2[:4],self.results3[:4])
        self.times.append(self.count/40.0)
        #self.min_distances.append(self.min_distance)
        self.count+=1
      
    def laser_callback(self,laser_scan):
        self.min_distance = min(laser_scan.ranges)

    #callback function that animates the plotting
    def animate(self,i,commands,times,dont,matter):
        # Limit x and y lists to window items
        self.results = self.results[-self.window:]
        self.times = self.times[-self.window:]

        self.axes.plot(self.times,self.results,color='red')
        self.axes.plot(self.times,self.results2,color='green')
        self.axes.plot(self.times,self.results3,color='blue')
        # Draw x and y lists
        #sns.lineplot(self.times,self.results,color='red', linewidth=2.5,ax=self.axes[0])
        #self.axes[0].spines['bottom'].set_color('#dddddd')
        #self.axes[0].spines['left'].set_color('#dddddd')
        sns.despine(left=False, bottom=False, right=True)
        # self.axes[0].tick_params(direction='out', length=6, width=2, colors="#5B5B5B",
        #        grid_color='r', grid_alpha=0.5,labelsize=12)
        # self.axes[0].set(ylim=(0, 1))
        # self.axes[0].set_yticks([0,1])
        # self.axes[0].set_title('Verification Result', color='#5B5B5B',loc='left',pad=25.0,fontweight="bold",fontsize=18)
        # self.axes[0].set_ylabel('Verification Result (Boolean)',fontsize=16,fontweight="bold",color='#5B5B5B')
        # self.axes[0].set_xlabel('Time (s)',fontweight="bold",color='#5B5B5B',fontsize=16)

        # sns.lineplot(self.times,self.min_distances,color='blue', linewidth=2.5,ax=self.axes[1])
        # self.axes[1].set_title('Minimum Distance to Obstacle (Lidar)', color='#5B5B5B',loc='left',pad=25.0,fontweight="bold",fontsize=18)
        # self.axes[1].set_ylabel('Distance (m)',fontsize=16,fontweight="bold",color='#5B5B5B')
        # self.axes[1].set_xlabel('Time (s)',fontweight="bold",color='#5B5B5B',fontsize=16)
        
if __name__=='__main__':
    rospy.init_node("reachability_result",anonymous=True)
    #get the arguments passed from the launch file
    res_node = PlotReachability()
    rospy.spin()
    