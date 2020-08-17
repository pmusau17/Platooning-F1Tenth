#!/usr/bin/env python
import rospy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np 
import time
from race.msg import drive_param
import os
import rospkg
import atexit
import matplotlib.pyplot as plt
import matplotlib.animation as animation


class AnalyzeData:
    ''' This class gather's data that will be used to perform greybox system identification of the F1Tenth Racecar in Matlab. 
        The model we will be using is the standard bicycle model that can be found here: (https://repository.upenn.edu/cgi/viewcontent.cgi?article=1908&context=cis_papers)

        The states of this model are [x,y,v,theta] where 
            x: x position 
            y: y position 
            v: linear velocity 
            theta: vehicle heading 

        It has two inputs: delta, u where: 
            delta: heading 
            u: throttle input (velocity setpoint in simulation)
    '''

    def __init__(self,racecar_name='racecar',vesc="vesc"):
        r = rospkg.RosPack() 
        # The data will be stored in a csv file in the csv directory
        self.odometry_sub=Subscriber(racecar_name+"/odom", Odometry)
        self.ackermann_stamped=Subscriber(vesc+'/ackermann_cmd_mux/input/teleop',AckermannDriveStamped)
        self.sub = ApproximateTimeSynchronizer([self.odometry_sub,self.ackermann_stamped], queue_size = 20, slop = 0.05)

        #register the callback to the synchronizer
        self.sub.registerCallback(self.master_callback)
        self.startTime = 0
        self.count = 0

        
        self.x = []
        self.y = []
        self.velocity = []
        self.times=[]
        self.theta=[]
        
        self.start_time=time.time()
        #figure for live animation
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(4, 1, 1)
        self.ax2 = self.fig.add_subplot(4, 1, 2)
        self.ax3 = self.fig.add_subplot(4, 1, 3)
        self.fig.tight_layout()
        self.window=4000

        #Animation
        # Set up plot to call animate() function periodically
        ani = animation.FuncAnimation(self.fig, self.animate, fargs=(self.theta, self.times,self.x,self.y,self.velocity), interval=1000)
        plt.show()


    #callback for the synchronized messages
    def master_callback(self,odom_msg,ackermann_msg): 
        

        # position 
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y

        # Convert Quaternion to rpy
        rpy = euler_from_quaternion([odom_msg.pose.pose.orientation.x,
                                     odom_msg.pose.pose.orientation.y,
                                     odom_msg.pose.pose.orientation.z,
                                     odom_msg.pose.pose.orientation.w])

        # linear velocity 
        velx = odom_msg.twist.twist.linear.x
        vely = odom_msg.twist.twist.linear.y
        velz = odom_msg.twist.twist.linear.z



        # magnitude of velocity 
        speed = np.asarray([velx,vely])
        speed = np.linalg.norm(speed)

       

        # heading 
        delta = ackermann_msg.drive.steering_angle

        # throttle 

        u = ackermann_msg.drive.speed

        # time 
        if(self.count == 0):
            self.startTime = rospy.Time.now()
            time = 0 
        else:
            time = rospy.Time.now()- self.startTime
            time = np.round(time.to_sec(),decimals=2)
        self.count+=1

        print("x:",x,"y:",y,"speed:",speed,"theta:",rpy[2],"u:",u,"delta:",delta,"time:",time)

        self.theta.append(rpy[2])
        self.times.append(time)
        self.x.append(x)
        self.y.append(y)
        self.velocity.append(speed)


    #function that animates the plotting
    def animate(self,i,times,theta,x,y,velocity):
        # Limit x and y lists to window items
        self.theta = self.theta[-self.window:]
        self.times = self.times[-self.window:]
        self.x = self.x[-self.window:]
        self.y = self.y[-self.window:]
        self.velocity = self.velocity[-self.window:]
        # Draw x and y lists
        self.ax.clear()
        self.ax2.clear()
        self.ax3.clear()

        self.ax.plot(self.times, self.theta)
        self.ax2.plot(self.times, self.velocity)
        self.ax3.plot(self.x, self.y)
        

        self.ax.set_title("theta vs time")
        self.ax2.set_title("velocity vs time")
        self.ax3.set_title("x and y position")
        

        # Format plot
        #plt.xticks(rotation=45, ha='right')
        #plt.subplots_adjust(bottom=0.30)
        #plt.title('Time (s) vs Vehicle heading (radians)')
        #plt.ylabel('Vehicle heading (radians)')
        #plt.xlabel('Time (s)')

        


if __name__ == '__main__':
    rospy.init_node('analyze_data')
    
    C = AnalyzeData()
    r = rospy.Rate(40)

    while not rospy.is_shutdown():
        r.sleep()
        

    
