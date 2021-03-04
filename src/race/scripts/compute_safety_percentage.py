#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32 
import numpy as np
import atexit
import rospkg 

class SafetyAggregator:
    def __init__(self,algorithm_name,velocity,experiment_number):
        self.safe_count = 0
        self.unsafe_count = 0
        self.start_time = None
        self.total_count = 0 
        self.algorithm_name = algorithm_name
        self.velocity = velocity
        self.experiment_number = experiment_number 
        # use the rospack object to get paths
        rospack = rospkg.RosPack()
        package_path=rospack.get_path('race')+"/logs/safety_experiments.csv"
        self.file = open(package_path, 'a')
        self.reachability_result=rospy.Subscriber('reachability_result',Float32,self.callback)
      

    def callback(self,result):
        res = result.data
        if(self.total_count==0):
            self.start_time = rospy.Time.now()
        if(res):
            self.safe_count+=1
        else:
            self.unsafe_count+=1
        self.total_count+=1


    def shutdown(self):
        safety_percentage = self.safe_count/float(self.total_count)
        self.file.write("{}, {}, {}, {}\n".format(self.experiment_number,self.algorithm_name,self.velocity,safety_percentage))
        self.file.close()

        
        
if __name__=='__main__':
    rospy.init_node("safety_aggregator",anonymous=True)
    #get the arguments passed from the launch file
    r = rospy.Rate(120)
    #get the arguments passed from the launch file
    args = rospy.myargv()[1:]

    # get the path to the file containing the waypoints
    algorithm_name = args[0]
    velocity = args[1]
    experiment_number = args[2]

    res_node = SafetyAggregator(algorithm_name,velocity,experiment_number)
    atexit.register(res_node.shutdown)
    while not rospy.is_shutdown():
        r.sleep()
    

    