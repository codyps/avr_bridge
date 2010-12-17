#! /usr/bin/env python
#http://www.ibm.com/developerworks/linux/tutorials/l-pysocks/section4.html
# http://www.sics.se/~adam/uip/uip-1.0-refman/
#  python structured data  http://www.doughellmann.com/PyMOTW/struct/

#This file contains the library files for 

import roslib; roslib.load_manifest('avr_bridge')
import rospy
import serial 
import struct
import yaml
import threading
import std_msgs.msg
import StringIO
import time

"""
At start up I need to read through the message definitions, and generate
"""
debug_packets = False

class Packet():
	""" This packet class is in charge of keep track of 
		transmission times, recieve/tranmission counts,
	"""
	def __init__(self, name, tag, msgConstructor):
		self.name = name
		
		self.tag  = tag
		self.type = 0
				
		self.total_trans = 0
		self.total_recieved = 0
		
		self.last_recv_time=None
		self.last_recv =None
		
		self._ackRecieved = None
		self.last_trans_time = None
		self.last_trans = None
		
		self.retransmissions = 0
		self.__MAX_RETRANSMISSIONS = 10
		
		self.msgConstructor = msgConstructor
		
	def parsePacketData(packet_data):
		"""
		Parses the recieved packet's data and 
		its msg object containing that data.
			@param packet_data :  raw binary string of packet data
			@param type packe_data : string
			@return : ros msg
			@rtype :  ros smg
		"""
		msg = self.msgConstructor()
		msg.deserialize(msg_data)
		self.last_recv = msg
		self.last_recv_time = time.Time()
		return msg
		
	def getMsgData(ros_msg):
		"""
		Deserializes a ros msg and keeps track of the transmission
		"""
		data_buffer = StringIO.StringIO()
		ros_msg.serialize(buffer)
		msg_data = buffer.getvalue()
		
		self.last_trans = ros_msg
		self.last_trans_time = time.Time()
		
		self._ackRecieved = False
		return msg_data

	def markRecieved()
		"""
			Marks that the last tranmissions was acknowledged
		"""
		self._ackRecieved = True
	def retransmit()
		""" 
			Checks to see if the last transmission should be resent
			because of acknowledgement timeout
		"""
		if ( (not self._ackRecieved) and 
			   ( (time.time() - self.last_trans_time) > 0.06):
			self.retransmissions += 1
			
			if (self.retransmissions > self.__MAX_RETRANSMISSIONS):
				error_msg = """%s retransmitted %d times.  FAIL"""
				raise Exception(error_msg%(self.name,self.retransmissions))
			return True
		else:
			return False
		

class AvrBridge():
	def __init__(self):
		
		self.port = None
		
		self.packets_name={} # packet types indexed by name
		self.packets_tag ={} # packet types indexed by tag #
		
		self.recieve_CBs={} # recieve call back functions
							# every time a recieved is called
							# its cb is called
							# keyed by tag
		
		self.io_thread = threading.Thread(target= self._check_io)
		self.io_thread.deamon = True
		
		#packet structures
		self.header_struct = struct.Struct('B B h') # packet_type topic_tag data_length

		
	def registerTopic(self, topic, msgConstructor, recv_cb = None):
		"""
			Registers a topic for transmission 
			@param topic : string of topic path/name
			@param msgConstructor : ros msg object
			@param 
		"""
		tag = len(self.packets_name) 
		packet = Packet(topic,tag, msgConstructor)
		self.packets_name[topic] =packet
		self.packets_tag[tag] = packet
		
		self.recieve_CBs[tag] = recv_cb
	def openPort(portName):
		self.port = serial.Serial(port, 57600, timeout=0.1)
		time.sleep(0.3)
		self.portName = port
		self.port.flushOutput()
		self.port.flushInput() 
		
	def run(self):
		self.io_thread.start()	
	def shutdown(self):
		self.done = True	
	def _io_thread(self):
		while not self.done:
		# check to see if there is new data
			self._check_io()
			
			#check to see if any packets failed and need
			#to be retransmitted
			#for name, packet in self.packet_name.iteritems():
			#	if packet.retransmit():
			#		msg = packet.last_trans
			#		self.send(name, msg)		
			time.sleep(0.01)
			
	def _check_io(self):
		"""
			Check the serial port and see if any new data has arived
		"""
		packet  = self._get_packet()
		if packet == None :
			return
		
		packet_data = packet # packet_data, checksum = packet
		
		#if not self._check_packet(packet_data, checksum) :
		#	#if the checksum does not match
		#	error_msg = "Checksum failed! :\n packet %s\n checksum %s\n"
		#	rospy.logdebug(error_msg%(packet_data, checksum) )
		#	return
		
		header, msg_data = packet_data[0:3], msg_data[4:]
		
		msg_type, tag, msg_len  = self.header_struct.unpack(header)
		
		if (msg_type == 0 ) : # part of publish/broadcast msg
			msg = self.packets_tag[tag].parsePacketData(msg_data)
			self.recieve_CBs[tag](msg)
		
	def _get_packet(self):
		"""
			Reads reads the packet and 
			returns the full binary string
			@return packet,checksum
			@rtype  binary strings
		"""
		if not self.port.isOpen():
			raise Exception("Cannot get packet data, the port is not open")

		header = self.port.read(4)
		
		if not (len(header) == 4) :
			return None
		
		packet_type, topic_tag, data_length = self.header_struct.unpack(header)
		msg_data = self.port.read(data_length) 
		return header+msg_data
	
	def is_port_ok():
		"""
		Check for error conditions on the port.
		"""
		if self.port == None:
			return False
		return True
		
	def send(topic, msg):
		"""
			This function sends the message over the serial port
			to the avr.
			@param topic : topic name
			@type  topic :  string
			@param msg   :  msg to be sent
			@type  msg   :  ros msg
		"""
		
		#grab the info for the header
		tag = self.packets_name[topic].tag
		data = self.packets_name[topic].getMsgData(msg)
		packet_type = self.packets_name[topic].type
		
		header_data = self.header_struct.pack(packet_type,tag,len(data))
		
		packet = header_data + data

		self.port.write(packet)
		self.port.flush()
	
	
	

class AvrBridgeNode():
	"""
		This class is responsible for autogenerating a basic
		AvrBridge node.  It reads a yaml configuration file
		and builds the node for you.
	"""
	def __init__(self):
		self.subscribers = {}
		self.publishers = {}
		self.portName = None
		
		self.config = None
		
		self.bridge = AvrBridge()
		
	def loadConfig(self, configFileObj):
		""" 
			@param configFileObj
			@type configFileObj : file like object
		"""
		self.config = yaml.load(configFileObj)
		
		#subscribes must get their topic ID first
		if self.config.has_key('subscribe'):
			for topic in self.config['subscribe']:
				#import that msg's python module
				msg_type = self.config['subscribe'][topic]['type']
				module_name,  msg_name = msg_type.split('/')
				
				try:
					module = __import__( module_name +'.msg')
				except:
					roslib.load_manifest(module_name)
					module = __import__( module_name +'.msg')

				
				msg_module = getattr(module, 'msg')
				msg = getattr(msg_module, msg_name)
							
				self.addSubscriber(topic, msg)
		
		
		if self.config.has_key('publish'):
			for topic in self.config['publish']:
				#import that msg's python module
				msg_type = self.config['publish'][topic]['type']
				module_name,  msg_name = msg_type.split('/')
				

				try:
					module = __import__( module_name +'.msg')
				except:
					roslib.load_manifest(module_name)
					module = __import__( module_name +'.msg')

				msg_module = getattr(module, 'msg')
				msg = getattr(msg_module, msg_name)
				
				self.addPublisher(topic, msg)
						 
	def addSubscriber(self, topic, msgConstructor):
		cb = lambda msg : self.bridge.send(topic, msg)
		self.subscribers[topic] = rospy.Subscriber(topic, msgConstructor,cb)
		
	def addPublisher(self, topic, msgConstructor):
		self.publishers[topic] = rospy.Pubisher(topic, msgConstructor)
		cb = lambda (msg) : self.publishers[topic].publish(msg)
		self.bridge.registerTopic(topic, msgConstructor, cb)
	
	def run(self):
		self.bridge.run()
		rospy.spin()
		self.bridge.shutdown()
