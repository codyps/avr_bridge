#!/usr/bin/env python
#

"""
AVR code generator for ROS topics.   

by Adam Stambler of Rutger University.

This software was written with support of a research grant (R01ES014717)
 from the National Institute of Environmental Health Sciences.  

"""
import roslib; roslib.load_manifest('avr_bridge')

import sys
import shutil
import os
import traceback

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
