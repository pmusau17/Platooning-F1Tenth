#!/usr/bin/env python
import rospy
from race.msg import velocity_msg

class SpeedManager:

    #default is to publish speed commands of 1 m/s at 40 hz 
    def __init__(self,racecar_name,rate=40.0,speed=1.0):
        self.rate=rate
        self.speed=speed
        self.pub=rospy.Publisher(racecar_name+'/velocity_msg',velocity_msg,queue_size=5)

    def publish(self):
        r = rospy.Rate(self.rate) # 40hz
        while not rospy.is_shutdown():
            msg=velocity_msg()
            msg.header.stamp=rospy.Time.now()
            msg.velocity=1.0
            self.pub.publish(msg)
            r.sleep()            

if __name__=="__main__":
    #get the arguments passed from the launch file 
    args = rospy.myargv()[1:]   
    racecar_name=args[0]
    rospy.init_node('speed_manager_'+racecar_name, anonymous=True)
    sp=SpeedManager(racecar_name)
    sp.publish()
    rospy.spin()
    

