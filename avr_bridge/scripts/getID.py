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
	#TODO   write something to check that this port exists
	
	try:
		bridge = avr_bridge.AvrBridge()
		bridge.openDevice(port)
	except Exception as e:
		print "could not access the port %s"%port
		print e;
		exit()
	import time
		
	bridge.run()
	time.sleep(2)
	avrID = bridge.getId()
	print "Avr Device %s has been connect to %s"%(avrID, port)
	print avrID
	
	bridge.shutdown()

	

