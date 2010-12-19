#! /usr/bin/env python
# ROS driver for Sparkfun 9degree of freedom AHRS
#
# microstrain stdev="0.00017"
# cov = (std*std)^2  (ie variance * variance)
# since the sparkfun imu cant be as good as the microstrain, i am going to say
# that the std = 0.01

import roslib
roslib.load_manifest('imu_9drazor')
import rospy
import serial
import tf
from threading import Thread
import math
import time
import avr_bridge

from sensor_msgs.msg  import Imu
from imu_9drazor.msg  import RazorImu

import sensor_msgs.msg

if __name__ == '__main__':
	rospy.init_node("razerIMU_msg_converter")
	
	imuMsg = Imu()
	imuMsg.orientation_covariance = [0.2 , 0 , 0,   0 , 0.2, 0,   0 , 0 ,0.2]
	imuMsg.angular_velocity_covariance = [0.2 , 0 , 0,   0 , 0.2, 0,   0 , 0 ,0.2]
	imuMsg.linear_acceleration_covariance = [0.2 , 0 , 0,   0 , 0.2, 0,   0 , 0 ,0.2]
		
	pub = rospy.Publisher('imu', Imu)
	
	
	def imuCB(rawMsg):
		imuMsg.angular_velocity.x  = rawMsg.angular_velocity.x
		imuMsg.angular_velocity.y  = rawMsg.angular_velocity.y
		imuMsg.angular_velocity.z  = rawMsg.angular_velocity.z
		
		imuMsg.linear_acceleration.x = rawMsg.linear_acceleration.x
		imuMsg.linear_acceleration.y = rawMsg.linear_acceleration.y
		imuMsg.linear_acceleration.z = rawMsg.linear_acceleration.z
		
		q =   tf.transformations.quaternion_from_euler(rawMsg.roll,rawMsg.pitch,rawMsg.yaw)
		imuMsg.orientation.x = q[1]
		imuMsg.orientation.y = q[1]
		imuMsg.orientation.z = q[2]
		imuMsg.orientation.w = q[3]

		pub.publish(imuMsg)
	
	sub = rospy.Subscriber('imu_raw', RazorImu, imuCB)
	
	print "Running IMU msg converter"
	
	rospy.spin()
	
	
	
	

