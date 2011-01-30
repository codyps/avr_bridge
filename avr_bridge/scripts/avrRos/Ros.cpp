/*
 * Ros.cpp
 *
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
 */

#include "Ros.h"
#include "string.h"

#include <stdio.h>

/* XXX: there are 3 ways to go about giving class Ros the ability to send
 * data over the wire:
 *  1) use hard coded function names which (for some inexplicable
 *     reason, use the stdio style even though it is uneeded)
 *  2) pass a (FILE *) to the Ros constructor.
 *  3) pass a class implimenting a send_packet method of some sort.
 *
 * Lets try to get #3 in here
 */
int ros_putchar(char c, FILE *stream);
int ros_getchar(FILE *stream);
static FILE *ros_io = fdevopen(ros_putchar, ros_getchar);

Ros::Ros(char const *node_name, uint8_t num_of_msg_types)
	: name(node_name)
	, in_ctx(num_of_msg_types)
{
	this->io = ros_io;
}

Ros::Ros(char const *node_name, uint8_t num_of_msg_types, FILE *_io)
	: io(_io)
	, name(node_name)
	, in_ctx(num_of_msg_types)
{}

RosInputCtx::RosInputCtx(uint8_t _topic_tag_max)
	: topic_tag_max(_topic_tag_max)
	, buffer_index(0)
{}

void Ros::subscribe(char const *topic, ros_cb funct, Msg *msg)
{
	int tag = getTopicTag(topic);
	this->cb_list[tag] = funct;
	this->msg_list[tag] = msg;
}

void Ros::publish(Publisher pub, Msg *msg)
{
	uint16_t bytes = msg->serialize(this->outBuffer);
	this->send_pkt(PT_TOPIC, pub, outBuffer, bytes);
}

void Ros::process_pkt()
{
	switch(this->in_ctx.header.packet_type) {
	case PT_GETID:
		this->getID();
		break;
	case PT_TOPIC:
		//ie its a valid topic tag
		//then deserialize the msg
		this->msg_list[this->in_ctx.header.topic_tag]->
			deserialize(this->in_ctx.buffer + 4);
		//call the registered callback function
		this->cb_list[this->in_ctx.header.topic_tag](this->
				msg_list[this->in_ctx.header.topic_tag]);
		break;
	case PT_SERVICE:
		break;
	}
}

bool RosInputCtx::append(char c)
{
	/* the last call to append completed the packet, start over */
	if (buffer_index == sizeof(this->header) + this->header.msg_length) {
		this->reset();
	}

	if (buffer_index == (ROS_BUFFER_SIZE - 1)) {
		this->reset();
	}

	this->buffer[this->buffer_index] = c;
	this->buffer_index++;

	bool header_completed = this->buffer_index == sizeof(this->header);
	bool packet_completed = this->buffer_index ==
		this->header.msg_length + sizeof(this->header);
	if (header_completed) {
		/* is the packet type something we know about? */
		if ((this->header.packet_type != PT_TOPIC) &&
				(this->header.packet_type != PT_GETID)) {
			this->reset();
			return false;
		}

		/* does the topic_tag make sense? */
		if (this->header.topic_tag >= this->topic_tag_max) {
			this->reset();
			return false;
		}

		/* does the msg_length make sense? */
		if (this->header.msg_length >= ROS_BUFFER_SIZE) {
			this->reset();
			return false;
		}

		return false;
	} else if (packet_completed) {
		return true;
	}
	return false;
}

void Ros::spin(char c)
{
	if (this->in_ctx.append(c)) {
		this->process_pkt();
	}
}

void Ros::spin()
{
	int com_byte = getc(this->io);

	while (com_byte != -1) {
		this->spin(com_byte);

		com_byte = getc(this->io);
	}
}

Publisher Ros::advertise(char const *topic)
{
	return getTopicTag(topic);
}

void Ros::send_pkt(uint8_t pkt_type, uint8_t topic,
		uint8_t const *data, uint8_t data_len)
{
	PktHeader head = {
		pkt_type,
		topic,
		data_len
	};

	fwrite(&head, sizeof(head), 1, this->io);

	fwrite(data, data_len, 1, this->io);
}

void Ros::getID()
{
	uint16_t size = this->name.serialize(this->outBuffer);
	this->send_pkt(PT_GETID, 0, outBuffer, size);
}

Ros::~Ros()
{}
