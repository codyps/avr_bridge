#!/usr/bin/env python
#

"""
AVR code generator for ROS topics.  This generates AVR source code 
so that the avr can communicate with the 

Converts ROS .msg files in a package into Python source code implementations.

arrays have an unsigned integer specifying the number of units in the array

"""
import roslib; roslib.load_manifest('avr_bridge')

import sys
import shutil
import os
import traceback

# roslib.msgs contains the utilities for parsing .msg specifications. It is meant to have no rospy-specific knowledge
import roslib.msgs 
import roslib.packages 


import roslib.genpy 
import yaml
import StringIO

from gen_avr_utils import *

if __name__ == "__main__":
	
	if (len(sys.argv) != 3 ):
		print """
		This program generates the c source code for an avr processor
		to communicate with ros over serial.
		
		It works with bridge_node.py
		
		To Use gen_avr.py
		./gen_avr.py configFile  outputDir
		"""
	
	config, outputDir = sys.argv[1:3]
  	
  	gen = CGenerator()
  	gen.parseConfig(open(config, 'r'))
  	gen.generate(outputDir)
