#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class PlotReachability:
    def __init__(self):
        self.reachability_result=rospy.Subscriber('reachability_result',Float32,self.callback)
        self.results=[]
        self.times=[]
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1, 1, 1)
        self.window=4000
        self.count= 0
        #Animation
        # Set up plot to call animate() function periodically
        ani = animation.FuncAnimation(self.fig, self.animate, fargs=(self.results, self.times), interval=1000)
        plt.show()

    def callback(self,result):
        res = result.data
        self.results.append(res)
        self.times.append(self.count/40.0)
        self.count+=1
        #function that animates the plotting
    
    def animate(self,i,commands,times):
        # Limit x and y lists to window items
        self.results = self.results[-self.window:]
        self.times = self.times[-self.window:]
        # Draw x and y lists
        self.ax.clear()
        self.ax.plot(self.times, self.results)

        # Format plot
        plt.xticks(rotation=45, ha='right')
        plt.subplots_adjust(bottom=0.30)
        plt.title('Reachability Result vs Time')
        plt.ylabel('Reachability Result (Boolean)')
        plt.xlabel('Time (s)')
        
if __name__=='__main__':
    rospy.init_node("reachability_result",anonymous=True)
    #get the arguments passed from the launch file
    res_node = PlotReachability()
    rospy.spin()
    