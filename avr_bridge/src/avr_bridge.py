#! /usr/bin/env python
"""
avr_bridge utilties 
by Adam Stambler of Rutger University.

This software was written with support of a research grant (R01ES014717)
 from the National Institute of Environmental Health Sciences.  


"""

import roslib; roslib.load_manifest('avr_bridge')
import rospy
import serial 
import struct
import yaml
import threading
import std_msgs.msg
import StringIO
import time
import Queue

debug_packets = False

class Packet():
	""" This packet class is in charge of keep track of 
		transmission times, recieve/tranmission counts.
		
		
	NOTE : in this version of the bridge, this tracking is not used
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
		
	def parsePacketData(self, packet_data):
		"""
		Parses the recieved packet's data and 
		its msg object containing that data.
			@param packet_data :  raw binary string of packet data
			@param type packe_data : string
			@return : ros msg
			@rtype :  ros smg
		"""
		msg = self.msgConstructor()
		msg.deserialize(packet_data)
		self.last_recv = msg
		self.last_recv_time = time.time()
		return msg
		
	def getMsgData(self, ros_msg):
		"""
		Deserializes a ros msg and keeps track of the transmission
		"""
		data_buffer = StringIO.StringIO()
		ros_msg.serialize(data_buffer)
		msg_data = data_buffer.getvalue()
		
		self.last_trans = ros_msg
		self.last_trans_time = time.time()
		
		self._ackRecieved = False
		return msg_data

	def markRecieved(self):
		"""
			Marks that the last tranmissions was acknowledged
		"""
		self._ackRecieved = True
	def retransmit(self):
		""" 
			Checks to see if the last transmission should be resent
			because of acknowledgement timeout
		"""
		if ( (not self._ackRecieved) and 
			   ( (time.time() - self.last_trans_time) > 0.06)):
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
		self.name = None #the name that the node gives itself
		
		self.packets_name={} # packet types indexed by name
		self.packets_tag ={} # packet types indexed by tag #
		
		self.recieve_CBs={} # recieve call back functions
							# every time a recieved is called
							# its cb is called
							# keyed by tag
		
		self.io_thread = threading.Thread(target= self._io_thread)
		self.io_thread.deamon = True
		
		self._done = threading.Event()

		
		self.queue = Queue.Queue()
		self.slock = threading.Semaphore()
		
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
	def openPort(self, portName):
		print "Port name is ", portName
		self.port = serial.Serial(portName, 57600, timeout=0.06)
		time.sleep(0.75)
		self.portName = portName
		self.port.flushOutput()
		self.port.flushInput() 
		self.port.flush()
		
	def run(self):
		"""
		Starts the io thread for the avr bridge
		"""
		if self.port == None :
			raise "Port not opened!"
		self.io_thread.start()	
		
		while not rospy.is_shutdown():
			while (not self.queue.empty() ):
				msg, t = self.queue.get()
				rospy.logdebug("topic : %s    msg:   %s"%(t,msg))
				self.sendAVR(msg, topic = t, rtype  = 0)
			rospy.sleep(0.01)
		self.shutdown()
		
		
	def shutdown(self):
		"""
		Turns off the io thread for avr_bridge and closes
		the serial port file.  This method must be called in order 
		for the program to exit cleanly.
		"""
		self._done.set()
		self.io_thread.join()
		self.port.close()
			
	def _io_thread(self):
		print "Beginning Communication with", self.portName
		while not self._done.isSet():
		# check to see if there is new data
			self._check_io()				
			time.sleep(0.01)
			
	def _check_io(self):
		"""
			Check the serial port and see if any new data has arived
		"""
		packet  = self._get_packet()
		if packet == None :
			return
		header, msg_data = packet[0:4], packet[4:]
		d = [ i for i in packet]
		#print "recieved ", d
		msg_type, tag, msg_len  = self.header_struct.unpack(header)
		
		try:
			if (msg_type == 0 ) : # part of publish/broadcast msg
				msg = self.packets_tag[tag].parsePacketData(msg_data)
				if (self.recieve_CBs[tag] != None):
					self.recieve_CBs[tag](msg)
			if (msg_type == 255): #getID message
				name = std_msgs.msg.String()
				name.deserialize(msg_data)
				self.name = name.data
		except Exception as e:
			d = [ i for i in packet]
			rospy.logdebug("Failed to deserialize packet %s", d)
			#print  e
			
		
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
		
		if (len(header) != 4) :
			return None
		
		packet_type, topic_tag, data_length = self.header_struct.unpack(header)
		
		msg_data = self.port.read(data_length) 
		if len(msg_data) != data_length:
			return None
		return header+msg_data
	
	def is_port_ok():
		"""
		Check for error conditions on the port.
		"""
		if self.port == None:
			return False
		return True
		
	def send(self, topic, msg):
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
		#self.port.flush()
		d = [i for i in packet]
		#print "sending ", d
		#rospy.logdebug("Sending packet %s", packet)
	def getId(self):
		t = 0
		self.port.write(self.header_struct.pack(255,0,0))
		while (self.name == None):
			time.sleep(0.04)
			t = t+1
			if (t >10):
				self.port.write(self.header_struct.pack(255,0,0))
			if (t >15):
				self.port.write(self.header_struct.pack(255,0,0))
			if (t > 20):
				return None
		return self.name
	
	

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
		
		self.portName = self.config['port']
		if not '/dev/' in self.portName:
			self.portName = '/dev/'+ self.portName
		
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
		"""
		Adds a subscription to the avr_bridge node.
		@param  topic : name of topic being published
		@param msgConstructor : msgConstructor for the topic's msg

		"""
		cb = lambda msg : self.bridge.send(topic, msg)
		self.subscribers[topic] = rospy.Subscriber(topic, msgConstructor,cb)
		self.bridge.registerTopic(topic, msgConstructor, None)
		
	def addPublisher(self, topic, msgConstructor):
		"""
		Adds a publisher to the bridge node
		@param  topic : name of topic being published
		@param msgConstructor : msgConstructor for the topic's msg
		"""
		self.publishers[topic] = rospy.Publisher(topic, msgConstructor)
		cb = lambda (msg) : self.publishers[topic].publish(msg)
		self.bridge.registerTopic(topic, msgConstructor, cb)
	
	def run(self):
		"""
		Start up basic bridge node
		"""
		self.bridge.openPort(self.portName)
		self.bridge.run()
		rospy.spin()
		self.bridge.shutdown()



