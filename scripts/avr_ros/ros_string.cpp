/*
 * ros_string.cpp
 *
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Adam Stambler
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Adam Stambler, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

#include "avr_ros/ros_string.h"
#include <string.h>
#include <stdlib.h>

using namespace ros;

string::string(){
	maxlength = 0;
}

string::string(MsgSz maxLength){
	maxLength=0;
	this->setMaxLength(maxLength);
}

void string::setMaxLength(MsgSz maxLength){
	if (this->maxlength <=0){
		data = (char*) malloc(maxLength+1);
		this->maxlength = maxLength;
		data[0]=0;
	}
}

string::string(char const *str){
	this->setString(str);
}

void string::setString(char const *str){
	MsgSz l = strlen(str);

	if (maxlength<=0) setMaxLength(l);

	l = (l > maxlength) ? maxlength : l;

	strncpy(this->data,str,l);
	this->data[l] = 0;
}

void string::serialize(PacketOut *p){
	MsgSz length = strlen(data);
	for(uint8_t i = 0; i < sizeof(length); i++) {
		p->pkt_send_byte((length >> i) & 0xff);
	}

	for(MsgSz i = 0; i < length; i++) {
		p->pkt_send_byte(data[i]);
	}
}

MsgSz string::deserialize(uint8_t* buffer){
	MsgSz length;
	memcpy(&length, buffer, sizeof(length));
	buffer += sizeof(length);
	//deal with the overflow quietly, just take as much as possible
	if (length > maxlength){
		memcpy(data, buffer, maxlength);
		data[maxlength] = 0;
	}
	else{
		memcpy(data, buffer, length);
		data[length] = 0;
	}

	return length + sizeof(length);
}

MsgSz string::bytes(){
	MsgSz length = strlen(data);
	return length + sizeof(length);
}

