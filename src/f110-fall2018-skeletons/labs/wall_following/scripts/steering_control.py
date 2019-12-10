#! /usr/bin/env python
"""
Author : Varundev Suresh Babu
Version: 0.1
"""

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan

steering_publisher = rospy.Publisher("steering_control", Float64, queue_size = 10)

angle_range = 270
car_length = 1.5
desired_trajectory = 0
vel = 15
path = [1,0,1,1,1]
turn = 0
preva = 0
flag = 0
errorList = [0,0,0,0,0]
turnStarted = 0
error = 0.0
alpha = 0.0
final_desired_trajectory = -1
final_direction = 1
left_dist = []
objbuffer = [-1]*25
flag_left = 0
kp = 10.0
kd = 0.01
kp_vel = 42.0
kd_vel = 0.0
ki = 0.0
servo_offset = 18.5
prev_error = 0.0
error = 0.0
integral = 0.0
vel_input = 25.0

def getRange(data,angle):
	if angle > 270:
		angle = 270
	index = len(data.ranges)*angle/angle_range
	dist = data.ranges[int(index)]
	if math.isinf(dist) or math.isnan(dist):
		return 10.0
	return data.ranges[int(index)]

def obs_particles(data, start, end, distance):
	global alpha
	front_values = []
	num_points = 0
	obs = 0
	alpha_degree = int(math.degrees(alpha))
	k = -1
	for i in range(start - alpha_degree, end - alpha_degree):
		k = k+1
		front_values.append(getRange(data,i)*math.sin(math.radians(i+alpha_degree)))
		if front_values[k] <= distance:
			num_points = num_points + 1
	return front_values,num_points

def obs_decide(data):
	global alpha
	start = 84
	end = 96
	distance = 2.0
	values,num_points = obs_particles(data,start,end,distance)
	print "In range", values
	start_point = 0
	end_point = 0
	print "Num Points", num_points
	if num_points < 3:
		print "Go go go - clear path"
		return -1,-1
	elif num_points <= 15:
		print "normal obstacle"
		k =-1
		for i in values:
			k = k + 1
			if i<= (distance):
				start_point = k + start
				break
		k = -1
		for i in reversed(values):
			k = k+1
			if i<= (distance):
				end_point = end - k
				break
		if start_point <= (start+1):
			print "Start point extended"
			start1 = start - 10
			end1 = start
			start_point = start1
			values,num_points = obs_particles(data,start1,end1,distance)
			print "Right extended", values
			k = 0
			for i in reversed(values):
				k = k + 1
				if i > (distance):
					start_point = end1 - k
					break
		if end_point >= (end-1):
			start2 = end + 1
			end2 = end + 10
			end_point = end2
			values,num_points = obs_particles(data,start2,end2,distance)
			print "Right extended", values
			k = len(values)-1
			for i in values:
				k = k-1
				if i > (distance):
					end_point = end2 - k
					break
		print "Start Point", start_point
		print "End Point", end_point
		return start_point,end_point
	else:
		print "wide obstacle"
		start1 = start - 10
		end1 = start - 1
		start_point = end1 + 3
		values,num_points = obs_particles(data,start1,end1,distance)
		k = len(values) - 1
		for i in reversed(values):
			k = k - 1
			if i > (distance):
				start_point = k + start1
				break
		start2 = end + 1
		end2 = end + 10
		end_point = start2 - 3
		values,num_points = obs_particles(data,start2,end2,distance)
		k = len(values)-1
		for i in values:
			k = k-1
			if i > (distance):
				end_point = end2 - k
				break
		print "wall"
		if start_point <= start1+1:
			start_point = -1
		if end_point >= end2-1:
			end_point = -1
		print "Start Point", start_point
		print "End Point", end_point
		return start_point,end_point

def decide_obstacle_direction(data,start_point,end_point):
	global alpha
	left = 0
	right = 1
	centre = 2
	stop = 3
	alpha_degree = int(math.degrees(alpha))
	desired_trajectory = -1
	direction = centre
	if start_point!=-1 or end_point!=-1:
		laser_start = getRange(data,start_point-alpha_degree)
		laser_end = getRange(data,end_point-alpha_degree)
		start_pointdistance = laser_start*math.cos(math.radians(start_point))
		end_pointdistance = laser_end*math.cos(math.radians(end_point))
		print "Right Width",start_pointdistance
		print "left Width",end_pointdistance
		car_dist_right = getRange(data,0)*math.cos(alpha)
		car_dist_left = getRange(data,179)*math.cos(alpha)
		print "Car dist right",car_dist_right
		print "Car dist left",car_dist_left
		obstacle_distance_left = 0.0
		obstacle_distance_right = 0.0
		obstacle_distance_right = car_dist_right - start_pointdistance
		obstacle_distance_left = car_dist_left + end_pointdistance
		print "Right edge",obstacle_distance_right
		print "left edge",obstacle_distance_left
		if (obstacle_distance_left > obstacle_distance_right):# and obstacle_distance_left > 0.4:
			desired_trajectory = obstacle_distance_left/2
			direction =left
			print "left"
		elif (obstacle_distance_left < obstacle_distance_right):# and obstacle_distance_right > 0.4:
			desired_trajectory = obstacle_distance_right/2
			direction =right
			print "right"
		else:
			direction = stop
	else:
		desired_trajectory = -1
		direction = centre
		print "Go centre"
	desired_trajectory = -1
	direction = centre
	print "Go centre"
	print "Desired dist", desired_trajectory
	return desired_trajectory, direction

def followRight(data,desired_trajectory):
	global alpha
	a = getRange(data,45)
	b = getRange(data,80)
	swing = math.radians(35)
	alpha = math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
	print "Alpha right",math.degrees(alpha)
	curr_dist = b*math.cos(alpha)
	future_dist = curr_dist+car_length*math.sin(alpha)
	print "Right : ",future_dist
	error = desired_trajectory - future_dist
	print "Error : ",error
	return error

def followLeft(data,desired_trajectory):
	global alpha
	a = getRange(data,190)
	b = getRange(data,225)
	swing = math.radians(35)
	alpha = -math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
	print "Alpha left",math.degrees(alpha)
	curr_dist = b*math.cos(alpha)
	future_dist = curr_dist-car_length*math.sin(alpha)
	print "Left : ",future_dist
	error = future_dist - desired_trajectory
	return error

def followCentre(data,desired_trajectory):
	global alpha
	a = getRange(data,115)
	b = getRange(data,155)
	swing = math.radians(40)
	alpha = -math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
	print "Alpha left",math.degrees(alpha)
	curr_dist = b*math.cos(alpha)
	future_dist1 = curr_dist-car_length*math.sin(alpha)
	a = getRange(data,50)
	b = getRange(data,0)
	swing = math.radians(50)
	alpha = math.atan((a*math.cos(swing)-b)/(a*math.sin(swing)))
	print "Alpha right",math.degrees(alpha)
	curr_dist = b*math.cos(alpha)
	future_dist2 = curr_dist+car_length*math.sin(alpha)
	desired_trajectory = (future_dist1 + future_dist2)/2
	print "dist 1 : ",future_dist1
	print "dist 2 : ",future_dist2
	error = future_dist1 - future_dist2
	print "Error : ",error
	return error

def decideReturn(start_point,end_point):
	global objbuffer
	objbuffer.append(start_point)
	objbuffer.append(end_point)
	del objbuffer[0]
	del objbuffer[0]
	if objbuffer.count(-1)==len(objbuffer):
		return 1
	else:
		return 0

def laser_callback(data):
	global error
	global alpha
	global flag_obstacle
	global final_desired_trajectory
	global final_direction
	global prev_direction
	global flag_left
        global integral
	global prev_error
	print integral
	global vel_input
	global kp
	global ki
	global kd
	global kd_vel
	global kp_vel
	velocity = vel_input
	angle = servo_offset
	start_point,end_point = obs_decide(data)
	desired_trajectory, direction = decide_obstacle_direction(data,start_point,end_point)
	if direction == 0:
		flag_left = 1
	elif direction == 2:
		flag_left = 0
	if flag_left == 1:
		direction = 0
	if direction == 0:
		final_desired_trajectory = 0.4
		final_direction = direction
	elif desired_trajectory != -1:
		final_desired_trajectory = desired_trajectory
		final_direction = direction
	car_dist_right = getRange(data,0)*math.cos(alpha)
	car_dist_left = getRange(data,179)*math.cos(alpha)
	if decideReturn(start_point,end_point) == 1:
		print "reset done"
		final_desired_trajectory = 0.8
		final_direction = 1
	print "Final Desired",final_desired_trajectory
	print "new code"
	if final_direction == 0:
		error = followLeft(data,final_desired_trajectory)
	else:
		error = followRight(data,final_desired_trajectory)
	final_desired_trajectory = 0.9
	error = followRight(data,final_desired_trajectory)
	error = 5.0*error
	print "Error Control",error
	if error!=0.0:
		if abs(error - prev_error)>0.5:
            integral = integral + error
		control_error = kp*error + kd*(prev_error - error)# + ki*integral
		integral = integral/1.3
		print "Control error",control_error
		angle = angle - control_error
	elif error == 0.0:
		angle = servo_offset
	prev_error = error
	if angle<-100:
		angle = -100
	if angle>100:
		angle = 100
	print "Angle",angle
	steer = Float64()
	steer.data = angle
	steering_publisher.publish(steer)

if __name__ == '__main__':
    rospy.init_node('steering_control_node')
    rospy.Subscriber("scan", LaserScan, laser_callback)
    rospy.spin()
