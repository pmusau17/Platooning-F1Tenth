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
sns.set(style='white', rc = {'legend.labelspacing': 1.2})

class PlotReachability:
    def __init__(self):
        self.reachability_result=rospy.Subscriber('reachability_result',Float32,self.callback)
        self.reachability_result=rospy.Subscriber('racecar2/scan',LaserScan,self.laser_callback)

        self.results=[]
        self.times=[]
        #self.fig,self.axes = plt.subplots(2, 1,figsize=(15,20))
        #self.fig,self.axes = plt.subplots(figsize=(20,15))
        #self.ax = self.fig.add_subplot(1, 1, 1)
        self.fig = plt.figure(figsize=(30,15))
        self.window=4000
        self.count= 0
        self.min_distances = []
        self.min_distance = np.inf
        #Animation
        # Set up plot to call animate() function periodically
        ani = animation.FuncAnimation(self.fig, self.animate, fargs=(self.results, self.times), interval=700)
        plt.show()


    def callback(self,result):
        res = result.data
        self.results.append(int(res))
        self.times.append(self.count/40.0)
        self.min_distances.append(self.min_distance)
        self.count+=1
      
    def laser_callback(self,laser_scan):
        self.min_distance = min(laser_scan.ranges)

    #callback function that animates the plotting
    def animate(self,i,commands,times):
        # Limit x and y lists to window items
        try:
            self.results = self.results[-self.window:]
            self.times = self.times[-self.window:]
            self.fig.clf()
            ax = sns.lineplot(self.times,self.results,color='red', linewidth=2.5,label='Verification Result')
            ax.spines['bottom'].set_color('#dddddd')
            ax.spines['top'].set_color('white')
            ax.set_title('Verification Result vs Distance to Obstacle (Lidar)', color='#5B5B5B',loc='left',pad=25.0,fontweight="bold",fontsize=30)
            ax.spines['left'].set_color('#dddddd')
            ax.set_ylabel('Verification Result (Boolean)',fontsize=25,fontweight="bold",color='#5B5B5B')
            ax.set_xlabel('Time (s)',fontweight="bold",color='#5B5B5B',fontsize=25)
            ax2= ax.twinx()
            ax2.set_ylabel('Distance (m)',fontsize=25,fontweight="bold",color='#5B5B5B')
            ax2.set_xlabel('Time (s)',fontweight="bold",color='#5B5B5B',fontsize=25)
            ax2.spines['bottom'].set_color('#dddddd')
            ax2.spines['top'].set_color('white')
            ax2.spines['left'].set_color('#dddddd')
            ax2.spines['right'].set_color('#dddddd')
            ax2 = sns.lineplot(self.times,self.min_distances,color='blue',label='Distance to Obstacle' ,linewidth=2.5,ax=ax2)
            ax.tick_params(direction='out', length=6, width=2, colors="#5B5B5B",
                grid_color='r', grid_alpha=0.5,labelsize=20)
            ax2.tick_params(direction='out', length=6, width=2, colors="#5B5B5B",
                grid_color='r', grid_alpha=0.5,labelsize=20)
            #plt.legend(None)
            lines, labels = ax.get_legend_handles_labels()
            lines2, labels2 = ax2.get_legend_handles_labels()
            ax2.legend(lines + lines2, labels + labels2, loc=0)
            plt.legend(None)
            #ax.legend(lines + lines2, labels + labels2, loc=0)
            sns.despine(left=False, bottom=False, right=False,top=True)

            
            
        except:
            pass

        # Draw x and y lists
        # sns.lineplot(self.times,self.results,color='red', linewidth=2.5,ax=self.axes[0])
        # self.axes[0].spines['bottom'].set_color('#dddddd')
        # self.axes[0].spines['left'].set_color('#dddddd')
        # sns.despine(left=False, bottom=False, right=True)
        # self.axes[0].tick_params(direction='out', length=6, width=2, colors="#5B5B5B",
        #        grid_color='r', grid_alpha=0.5,labelsize=14)
        # self.axes[0].set(ylim=(0, 1))
        # self.axes[0].set_yticks([0,1])
        # self.axes[0].set_title('Verification Result', color='#5B5B5B',loc='left',pad=25.0,fontweight="bold",fontsize=20)
        # self.axes[0].set_ylabel('Verification Result (Boolean)',fontsize=18,fontweight="bold",color='#5B5B5B')
        # self.axes[0].set_xlabel('Time (s)',fontweight="bold",color='#5B5B5B',fontsize=18)

        # sns.lineplot(self.times,self.min_distances,color='blue', linewidth=2.5,ax=self.axes[1])
        # self.axes[1].spines['bottom'].set_color('#dddddd')
        # self.axes[1].spines['left'].set_color('#dddddd')
        # self.axes[1].set_title('Minimum Distance to Obstacle (Lidar)', color='#5B5B5B',loc='left',pad=25.0,fontweight="bold",fontsize=20)
        # self.axes[1].set_ylabel('Distance (m)',fontsize=18,fontweight="bold",color='#5B5B5B')
        # self.axes[1].set_xlabel('Time (s)',fontweight="bold",color='#5B5B5B',fontsize=18)
        # self.axes[1].tick_params(direction='out', length=6, width=2, colors="#5B5B5B",
        #        grid_color='r', grid_alpha=0.5,labelsize=14)
        
if __name__=='__main__':
    rospy.init_node("reachability_result",anonymous=True)
    #get the arguments passed from the launch file
    res_node = PlotReachability()
    rospy.spin()
    