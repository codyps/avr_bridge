#!/usr/bin/env python
#

"""
AVR code generator for ROS topics.   

by Adam Stambler of Rutger University.

This software was written with support of a research grant (R01ES014717)
 from the National Institute of Environmental Health Sciences.  


# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Adam Stambler
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Adam Stambler, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
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
                usage: gen_avr.py <msg_spec> <output_dir>

		Generate C++ code implimenting message serialization
                and deserialization.

                Designed for use with avr_bridge.py
		"""
	
	config, outputDir = sys.argv[1:3]
  	
  	gen = CGenerator()
  	gen.parseConfig(open(config, 'r'))
  	gen.generate(outputDir)
