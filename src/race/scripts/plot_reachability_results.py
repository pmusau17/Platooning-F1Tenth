#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import seaborn as sns 
import numpy as np

class PlotReachability:
    def __init__(self):
        self.reachability_result=rospy.Subscriber('reachability_result',Float32,self.callback)
        self.results=[]
        self.times=[]
        self.fig = plt.figure(figsize=(15,8))
        #self.ax = self.fig.add_subplot(1, 1, 1)
        self.window=4000
        self.count= 0
        #Animation
        # Set up plot to call animate() function periodically
        ani = animation.FuncAnimation(self.fig, self.animate, fargs=(self.results, self.times), interval=700)
        plt.show()

    def callback(self,result):
        res = result.data
        self.results.append(int(res))
        self.times.append(self.count/40.0)
        self.count+=1
        #function that animates the plotting
    
    def animate(self,i,commands,times):
        # Limit x and y lists to window items
        self.results = self.results[-self.window:]
        self.times = self.times[-self.window:]
        # Draw x and y lists
        ax = sns.lineplot(self.times,self.results,color='red', linewidth=2.5)
        ax.spines['bottom'].set_color('#dddddd')
        ax.spines['left'].set_color('#dddddd')
        sns.despine(left=False, bottom=False, right=True)
        ax.set(ylim=(0, 1))
        ax.set_yticks([0,1])
        ax.set_title('Reachability Result vs Time', color='#5B5B5B',loc='left',pad=25.0,fontweight="bold",fontsize=14)
        plt.ylabel('Reachability Result (Boolean)',fontsize=12,fontweight="bold",color='#5B5B5B')
        plt.xlabel('Time (s)',fontweight="bold",color='#5B5B5B',fontsize=12)
        
if __name__=='__main__':
    rospy.init_node("reachability_result",anonymous=True)
    #get the arguments passed from the launch file
    res_node = PlotReachability()
    rospy.spin()
    