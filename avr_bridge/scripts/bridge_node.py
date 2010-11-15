#! /usr/bin/env python

#  Rutgers IEEE AVR Bridge tool
#  
#  This tool reads in the configuration, opens the serial port
#  and acts as a bridge between the avr and ROS.  It allows 
#  the avr to indirectly publish the ros message types 
#

import roslib; roslib.load_manifest('avr_bridge')
import avr_bridge
import rospy
import sys


if __name__ == '__main__':
	rospy.init_node('bridge_node')
	print "\n ****  avr_bridge's bridge_node.py   *****\n"
	print "Opening file ", sys.argv[1]
	
	configFile = sys.argv[1]
	
	bridge = avr_bridge.AvrBridge(open(configFile,'r'))
	for topic in bridge.publishers.keys():
		print "Publishing on Topic : ", topic
	for topic in bridge.subscribers.keys():
		print "Subscribing to Topic : ", topic
	 
	bridge.run()
	rospy.spin()
	bridge.shutdown()
