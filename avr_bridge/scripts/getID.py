#! /usr/bin/env python

#  Rutgers IEEE AVR Bridge tool
#  
#
#

import roslib; roslib.load_manifest('avr_bridge')
import avr_bridge

import sys

if __name__ == '__main__':
	#contacts device using the bridge
	#and gets the ID
	
	
	port = sys.argv[1]

	try:
		bridge = avr_bridge.AvrBridge()
		bridge.openPort(port)
	except Exception as e:
		print "could not access the port %s"%port
		print e;
		exit()
	import time
		
	bridge.run()

	avrID = bridge.getId()
	
	if avrID == None:
		print "No avr_bridge device has been attached to ", sys.argv[1]
		print sys.argv[1]
	else:	 
		print "Avr Device %s has been connect to %s"%(avrID, port)
		print avrID
		
	bridge.shutdown()

	

