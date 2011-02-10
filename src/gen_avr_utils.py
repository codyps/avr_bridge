#!/usr/bin/env python
# vim: sw=8 ts=8 sts=8 noet 
"""
AVR code generator for ROS topics.  This utilities are used to generate
the avr source code to communicate with the ros avr bridge.  

by Adam Stambler of Rutger University.

Written with support of a research grant (R01ES014717)
from the National Institute of Environmental Health Sciences.

Software License Agreement (BSD License)

Copyright (c) 2011, Adam Stambler
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

 * Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above
   copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided
   with the distribution.
 * Neither the name of Adam Stambler, Inc. nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""
import roslib; roslib.load_manifest('avr_bridge')

import sys
import shutil
import os
import traceback

import roslib.msgs 
import roslib.packages 

import roslib.genpy 
import yaml
import StringIO

from c_writer import SimpleStateC

primitives = {
	'bool'  :   ('bool',     1),
	'byte'  :   ('uint8_t',  1),
	'int8'  :   ('uint8_t',  1),
	'int16' :   ('int16_t',  2),
	'uint16':   ('uint16_t', 2),
	'int32':    ('int32_t',  4),
	'uint32':   ('uint32_t', 4),
	'int64':    ('int64_t',  8),
	'uint64':   ('uint64_t', 8),
	'float32':  ('float',    4),
	'float64':  ('double',   8),
	'time':     ('int64_t',  8),
	'duration': ('int64_t',  8),
	'string' :  ('ros::string', 0)
}

def extract_ros_type(field):
	"""
	extract the basic type
	from the msg_spec field
	without the array symbols
	and with the pkg separted
	"""
	try:
		incPkg, incName = field.type.split('/')
	except:
		incPkg = None
		incName = field.type
	if field.is_array:
		incName = incName[:incName.find('[')]
	return incPkg, incName
	
def type_to_rint(t):
	conv = {
		1 : 'uint8_t',
		2 : 'uint16_t',
		4 : 'uint32_t',
		8 : 'uint64_t'
	}

	return conv[t]

def make_union_cast(cf, field_name, otype, olen):
	uname = 'u_{0}'.format(field_name)

	cf.line('union {')
	cf.indent()
	cf.line('{0} real;'.format(otype))
	cf.line('{0} base;'.format(type_to_rint(olen)))
	cf.dedent()
	cf.line('}} {0};'.format(uname))

	return uname

def serialize_primitive(f, buffer_addr, field):
	"""Generate c code to serialize rostype of fieldname at the buffer 
	"""
	fpkg, ftype = extract_ros_type(field)
	ctype, clen = primitives[ftype]
	fname = field.name
	
	if (field.is_array or field.type == 'string'):
		f.line('offset += this->{0}.serialize('.format(fname) +
		       '{0} + offset);'.format(buffer_addr))
	else:
		this = make_union_cast(f, field.name, ctype, clen)
		f.line('{0}.real = this->{1};'.format(this, fname))
		for byte in range(0, clen):
			mask = '0xFF'
			f.line('*({0} + offset + {1}) = '.format(buffer_addr,
								byte)
			     + '({0}.base >> (8 * {1})) & {2};'.format(
					this, byte, mask))
		f.line('offset += sizeof(this->{0});'.format(fname)) 

def deserialize_primitive(f, buffer_addr, field):
	"""
	Generate c code to deserialize a rosmsg field of type ctype from 
	specified buffer.
	"""
	fpkg, ftype = extract_ros_type(field)
	ctype, clen = primitives[ftype]
	fname = field.name

	if (field.is_array or field.type == 'string'):
		f.line('offset += this->{0}.deserialize('.format(fname) +
		       '{0} + offset);'.format(buffer_addr))
	else:
		this = make_union_cast(f, fname, ctype, clen)
		f.line('{0}.base = 0;'.format(this))
		for byte in range(0, clen):
			ol = '{0}.base |= ((typeof({0}.base)) (*({1} + offset + {2}))) << (8 * {3});' 
			f.line(ol.format(this, buffer_addr, byte, byte))

		f.line('this->{0} = {1}.real;'.format(fname, this))
		f.line('offset += sizeof(this->{0});'.format(fname))
				

def write_header_file(f, msg_name, pkg, msg_spec):
	"""
	f is a file like object to which the header file is being outputed
	msg_name is the name of the msg
	pkg is the msg's pkg
	includes is a list of file names that should be included
	filds is a list of touples of field type, name, array, array_length
	"""
	#write comments
	f.line('/* {0}.h'.format(msg_name))
	f.line(' * MSG file auto generated by "Rutgers avr_bridge"')
	f.line(' */')
	
	#write header guards
	guard = msg_name+'_H_'
	guard = guard.upper()
	f.macro_line('ifndef {0}'.format(guard))
	f.macro_line('define {0}'.format(guard))

	f.macro_line('include "Msg.h"')
	f.macro_line('include "vector.h"')
	f.macro_line('include "ros_string.h"')

	#write includes
	for field in msg_spec.parsed_fields():
		if not field.is_builtin:
			incPkg, incName = extract_ros_type(field)
			f.macro_line('include "{0}.h"'.format(incName))
	
	#open namespace

	f.line('namespace {0} {{'.format(pkg))
	f.indent()

	f.line('class {0} : public Msg {{'.format(msg_name))
	f.line('public:')
	f.indent()

	f.line('MsgSz bytes();')
	f.line('MsgSz serialize(uint8_t *out_buffer);')
	f.line('MsgSz deserialize(uint8_t *data);')
	
	#write msg fields
	for field in msg_spec.parsed_fields():
		fpkg, ftype = extract_ros_type(field)
		if fpkg != None:
			ftype = fpkg +"::"+ftype
		if ftype == 'Header':
			ftype = "roslib::"+ftype
		if field.is_builtin:
			ftype, clen = primitives[ftype]
		if field.is_array:
			if field.array_len:
				f.line('ros::vector<{0}> {1}; // fixed at length {2}'.format(ftype, field.name, field.array_len))
			else:
				f.line('ros::vector<{0}> {1};'.format(ftype, field.name))
		else:
			f.line('{0} {1};'.format(ftype, field.name))
	
	f.dedent()
	f.line('}}; /* class {0} */'.format(msg_name))

	#close namespace
	f.dedent()
	f.line('}} /* namespace {0} */'.format(pkg))
	#closing header guards
	f.macro_line('endif /* {0} */'.format(guard))

def serialize_msg(f, msg_spec):
	f.line('MsgSz offset = 0;')
	
	for field in msg_spec.parsed_fields():
		if (field.is_builtin):
			serialize_primitive(f, 'data', field)
		elif (field.is_array and field.array_len):
			serialize_farray(f, 'data', field)
		else:
			f.line('offset += this->{0}.serialize({1} + offset);'.format(field.name, 'data'))
			
	f.line('return offset;')

def deserialize_msg(f, msg_spec):
	f.line('MsgSz offset = 0;')
	
	for field in msg_spec.parsed_fields():
		if (field.is_builtin):
			deserialize_primitive(f, 'data', field)
		else:
			f.line('offset += this->{0}.deserialize({1} + offset);'.format(field.name, 'data'))
			
	f.line('return offset;')



def w_msg_fn_bytes(f, msg_spec):
	"""
	write out the bytes() member function for a msg.  
	iterate through the msg fields and extracts either their size if 
	they are a primitive type, or call the bytes() of that msg.

	@param f :  output file object
	@param msg_spec : the msg_spec of the msg
	"""
	f.line('MsgSz msgSize = 0;')
	
	for field in msg_spec.parsed_fields():
		if (field.is_builtin and not (field.type == 'string') ):
			fpkg, ftype = extract_ros_type(field)
			ctype, clen = primitives[ftype]
			f.line('msgSize += sizeof({0});'.format(ctype))
		else:
			f.line('msgSize += {0}.bytes();'.format(field.name))
	
	f.line('return msgSize;')
	

def write_cpp(f, msg_name, pkg, msg_spec):
	"""
	Generate the msg cpp implementation file
	@param f : output file object
	@param msg_name : name of msg type
	@param pkg : pkg that the message is found in
	@param msg_spec : msg_spec object of the msg
	"""

	f.macro_line('include "{0}.h"'.format(msg_name))
	f.macro_line('include <ros.h>')
	f.macro_line('include <stdio.h>')
	
	f.line('using namespace {0};'.format(pkg))
	
	def writeFunct(rtype, msg, funct, args, implementation):
		if rtype != '':
			f.line('{0} {1}::{2}({3})'.format(rtype,msg,funct,args))
		else:
			f.line('{0}::{1}({2})'.format(msg, funct, args))
		f.line('{')
		f.indent()
		implementation(f)
		f.dedent()
		f.line('}')
	
	#write constructor
	#set fixed length arrays
	constructor_init=''
	for field in msg_spec.parsed_fields():
		if field.is_array:
			if field.array_len:
				constructor_init += '%s(%d),' %(field.name, field.array_len)
	if (len(constructor_init) > 3):
		f.line('{0}::{0}()'.format(msg_name))
		f.line( ':' + constructor_init[:-1])
		f.line('{}')
	
	writeFunct('MsgSz', msg_name, 'serialize', 'uint8_t *data', lambda f: serialize_msg(f, msg_spec))
	writeFunct('MsgSz', msg_name, 'deserialize', 'uint8_t *data', lambda f: deserialize_msg(f,msg_spec))
	writeFunct('MsgSz', msg_name, 'bytes', '', lambda f: w_msg_fn_bytes(f, msg_spec))

class CGenerator():
	"""
	Class responsible for generating the c++ files from the yaml 
	configuration file.
	"""
	def __init__(self):
		self.types = [] #contains list of distinct types 
						#each type will generate a .h and .cpp file
		self.msg_specs = {} #dict containing the msg spec for each type
		
		self.topicIds = {}
		self._topicIds = 0
		
		self.config = None
		
	def parseConfig(self, configFile):
		"""
			Takes a file like object of the yaml configuration 
		"""
		
		self.config = yaml.load(configFile)
		
		#services get their topic id first
		if self.config.has_key('service'):
			for topic in self.config['service']:
				#import that msg's python module
				msg_type = self.config['service'][topic]['type']

			#TODO IMPLEMENT SERVICES

				self.topicIds[topic] = self._topicIds
				self._topicIds +=1
											
		
		#subscribes must get their topic id first
		if self.config.has_key('subscribe'):
			for topic in self.config['subscribe']:
				#import that msg's python module
				msg_type = self.config['subscribe'][topic]['type']
								
				self.addMsg(topic, msg_type)
				
				self.topicIds[topic] = self._topicIds
				self._topicIds +=1

		
		if self.config.has_key('publish'):
			for topic in self.config['publish']:
				#import that msg's python module
				msg_type = self.config['publish'][topic]['type']
	
				self.addMsg(topic, msg_type)
				
				self.topicIds[topic] = self._topicIds
				self._topicIds +=1
		
	def addMsg(self, pkg, name):
		"""
		@param pkg  The package that the msg is found in
		@param msg  The name of the msg 
		"""
				
		msg_name, msg_spec = roslib.msgs.load_by_type(name, pkg)
		if not (msg_name in self.types):
			self.types.append(msg_name)
			self.msg_specs[msg_name] = msg_spec
			
			#now that we are done adding the base msg type, we need to
			#recursively add all the types defined within it
			for msgType in msg_spec.types:
				if msgType[-1] == ']': #strip away array markers
					msgType = msgType[0:msgType.find('[')]

				if msgType == 'Header':
					self.addMsg('roslib', 'Header')
				elif primitives.has_key(msgType) or msgType== 'string':
					pass
				else:
					print "The msg type is ", msgType
					tpkg, tmsg = msgType.split('/')
					self.addMsg(tpkg, tmsg)


	def generateRos(self, folderPath):
		""" 
	This function generates to ros_generated.cpp file.
	This file contains the getTopicTag implementation along with
	the instantiation of the ros object.
		"""
		fo = open(folderPath+'/GenRos.h', 'w');
		
		f = SimpleStateC(fo)

		f.line('/* This file was autogenerated as a part of the avr_bridge pkg')
		f.line(' * avr_bridge was written by Adam Stambler and Phillip Quiza of')
		f.line(' * Rutgers University.')
		f.line(' */')

		f.macro_line('include <ros.h>')

		msg_ct = len(self.topicIds)

		f.line('char Ros<{0}>::getTopicTag(char *topic) {{'.format(msg_ct))
		f.indent()

		for topic_name, topic_id in self.topicIds.iteritems():
			f.line('if (!strcmp(topic, "{0}"))'.format(topic_name))
			f.indent()
			f.line('return {0};'.format(topic_id))
			f.dedent()


		f.line('return 0;')
		f.dedent()

		f.line('}')

		f.line('Ros <{1}> ros("{0}");'.format(self.config['name'], msg_ct))
		f.close()
		
		
	def generateMsgFile(self, folderPath, msg):
		pkg, msg_name = msg.split('/')
		
		header_file = open(folderPath+'/'+msg_name+'.h', 'w')
		f = SimpleStateC(header_file)
		write_header_file(f, msg_name, pkg, self.msg_specs[msg])
		header_file.close()
	
		cpp_file = open(folderPath+'/'+msg_name+'.cpp','w')
		f = SimpleStateC(cpp_file)
		write_cpp(f, msg_name, pkg, self.msg_specs[msg])
		
	def generate(self, folder_path):
		""" generate the ros implementation for the avr code
		"""
		genPath = roslib.packages.find_resource(
				'avr_bridge' ,'gen_avr.py')[0]
		avrRosPath =  genPath[:-len('gen_avr.py')]+ 'avrRos'
		inst_path = folder_path + '/avrRos'

		try:
			shutil.copytree(avrRosPath, inst_path)
		except Exception as e:
			print "avrRos already exists in ", inst_path
			print "The new files are overwriting the old files"
			shutil.rmtree(inst_path)
			shutil.copytree(avrRosPath, inst_path)
		
		for t in self.types:
			self.generateMsgFile(inst_path, t)
		
		self.generateRos(inst_path)
		
	
	
def test():
	roslib.msgs.set_verbose(False)
    
	typeList = []
	
	msg_name = 'Odometry'
	pkg_name = 'nav_msgs'
	msg_path = roslib.msgs.msg_file(pkg_name, msg_name)
	msg_name, msg_spec = roslib.msgs.load_from_file(msg_path)
	
	gen = CGenerator()
	gen.addMsg(pkg_name, msg_name)
	 
