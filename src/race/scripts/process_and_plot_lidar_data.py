#!/usr/bin/env python
from sensor_msgs.msg import LaserScan
import time
import rospy
import copy
import numpy as np
import math


"""The goal of this node is to display the lidar data being used by the F1Tenth car as a sanity check
The sensormsgs/LaserScan message has the following fields

Header header            # timestamp in the header is the acquisition time of 
                         # the first ray in the scan.
                         #
                         # in frame frame_id, angles are measured around 
                         # the positive Z axis (counterclockwise, if Z is up)
                         # with zero angle being forward along the x axis
                         
float32 angle_min        # start angle of the scan [rad]
float32 angle_max        # end angle of the scan [rad]
float32 angle_increment  # angular distance between measurements [rad]

float32 time_increment   # time between measurements [seconds] - if your scanner
                         # is moving, this will be used in interpolating position
                         # of 3d points
float32 scan_time        # time between scans [seconds]

float32 range_min        # minimum range value [m]
float32 range_max        # maximum range value [m]

float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
float32[] intensities    # intensity data [device-specific units].  If your
                         # device does not provide intensities, please leave
                         # the array empty.

"""


pub = rospy.Publisher('processed_data', LaserScan, queue_size=5)
pub2 = rospy.Publisher('processed_data_behind_car', LaserScan, queue_size=5)


""" Scans each pair of subsequent values, and returns an array of indices
        where the difference between the two values is larger than the given
        threshold. The returned array contains only the index of the first value
        in pairs beyond the threshold. """

def find_disparities(arr,threshold):
      
        to_return = []
        values = arr
        #print("Why would you consider disparities behind the car",len(values))
        for i in range(400,901):
            if abs(values[i] - values[i + 1]) >= threshold:
                #print("disparity: ",(values[i], values[i + 1]))
                #print("indices: ",(i, i + 1))
                to_return.append(i)
        return to_return


"""Computes the target angle to drive at"""
def sample_distance(arr):
    if(len(arr)==1):
        return arr[0]
    else:
        mid=int(len(arr)/2)
        return arr[mid]

def calculate_samples_based_on_arc_length(distance,car_width):
        """ Returns the number of points in the LIDAR scan that will cover half of
        the width of the car along an arc at the given distance. """
        # This isn't exact, because it's really calculated based on the arc length
        # when it should be calculated based on the straight-line distance.
        # However, for simplicty we can just compensate for it by inflating the
        # "car width" slightly.

         #store the value of 0.25 degrees in radians
        angle_step=(0.25)*(math.pi/180)
        arc_length=angle_step*distance
        return int(math.ceil(car_width / arc_length))

def extend_disparities(arr,disparity_indices,car_width):
    ranges=np.copy(arr)
    for i in disparity_indices:
        #get the values corresponding to the disparities
        value1=ranges[i]
        value2=ranges[i+1]
        #Depending on which value is greater we either need to extend left or extend right
        if(value1<value2):
            nearer_value=value1
            nearer_index=i
            extend_positive=True
        else:
            nearer_value=value2
            extend_positive=False
            nearer_index=i+1
        #compute the number of samples needed to "extend the disparity"
        samples_to_extend=calculate_samples_based_on_arc_length(nearer_value,car_width)
        #print("Samples to Extend:",samples_to_extend)

        #loop through the array replacing indices that are larger and making sure not to go out of the specified regions   
        current_index = nearer_index
        for i in range(samples_to_extend):
                # Stop trying to "extend" the disparity point if we reach the
                # end of the array.
                if current_index < 180:
                    current_index = 180
                    break
                if current_index >=901:
                    current_index =900
                    break
                # Don't overwrite values if we've already found a nearer point
                if ranges[current_index] > nearer_value:
                    ranges[current_index] = nearer_value
                # Finally, move left or right depending on the direction of the
                # disparity.
                if extend_positive:
                    current_index += 1
                else:
                    current_index -= 1
    return ranges

"""This function returns the angle we are targeting depending on which index corresponds to the farthest distance"""
def calculate_angle(index):
    angle=(540-index)/4.0
    rad=(angle*math.pi)/180
    print(angle,rad)
    return rad

"Threshold the angle if it's larger than 35 degrees"
def threshold_angle(angle):
    max_angle_radians=35*(math.pi/180)
    if angle<(-max_angle_radians):
        return -max_angle_radians
    elif angle>max_angle_radians:
        return max_angle_radians
    else:
        return angle




def lidar_callback(data):
    global pub
    global pub2
    #copy the scan currently being published
    new_scan=copy.copy(data)
    ranges=data.ranges
    #convert the range to a numpy array so that we can process the data
    limited_ranges=np.asarray(ranges)
    #ignore everything outside the -90 to 90 degree range
    limited_ranges[0:400]=0.0
    limited_ranges[901:]=0.0
    #add this so that the last element is not detected as a disparity
    limited_ranges[901]=limited_ranges[900]
    indices=np.where(limited_ranges>=10.0)[0]
    limited_ranges[indices]=(data.range_max)-0.1

    #calculate the disparities between samples
    threshold=0.50
    car_width=0.25
    disparities=find_disparities(limited_ranges,threshold)
    
    #go through the disparities and extend the disparities 
    new_ranges=extend_disparities(limited_ranges,disparities,car_width)

    #set the new computed ranges and publish it so you can see it in rviz
    new_scan.ranges=new_ranges
    #reset the time stamp
    new_scan.header.stamp=rospy.Time.now()
    #compute the max_value of the new limited values
    max_value=max(new_scan.ranges)
    target_distances=np.where(new_ranges>=max_value)[0]
    
    driving_distance=sample_distance(target_distances)
    driving_angle=calculate_angle(driving_distance)
    thresholded_angle=threshold_angle(driving_angle)
    print("Max Value:",max_value)
    print("Driving Distance Index:",driving_distance,"Computed Angle:",driving_angle,"Thresholded Angle:",thresholded_angle,"Distance at that angle:",new_ranges[driving_distance])
    pub.publish(new_scan)

    behind_car=np.asarray(data.ranges)
    behind_car[180:901]=0.0
    #set the new computed ranges and publish it so you can see it in rviz
    new_scan.ranges= behind_car
    #reset the time stamp
    new_scan.header.stamp=rospy.Time.now()
    pub2.publish(new_scan)
    
    


if __name__=="__main__":
    rospy.init_node('plot_lidar_data', anonymous=True)
    rospy.Subscriber('scan', LaserScan, lidar_callback)
    rospy.spin()

