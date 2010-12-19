import roslib; roslib.load_manifest('avr_bridge')
import rospy

import avr_bridge
import serial 
import struct
import yaml
import threading
import std_msgs.msg
import StringIO
import time
