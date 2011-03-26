/*
 * Ros.h
 *
 * Software License Agreement (BSD License)
 *
 * Copyright 2011, Cody Schafer <cpschafer@gmail.com>
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

#ifndef NODE_HANDLE_H_
#define NODE_HANDLE_H_

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avr_ros/types.h"
#include "avr_ros/string.h"
#include "avr_ros/packet_out.h"
#include "avr_ros/msg.h"
#include "avr_ros/byte_io.h"

#define __deprecated __attribute__((deprecated))

#ifndef UINT8_MAX
#define UINT8_MAX 0xff
#endif

namespace ros {

typedef void (RosCb)(Msg const *msg);

typedef uint8_t Publisher;

template <size_t MSG_CT, size_t BUFFER_SZ>
struct InputCtx {
	InputCtx()
		: buffer_index(0)
	{}

	bool append(char c)
	{
		/* out of space. */
		if (buffer_index == BUFFER_SZ) {
			this->reset();
		}

		this->buffer[this->buffer_index] = c;
		this->buffer_index++;

		bool header_completed = this->buffer_index ==
			sizeof(this->header);
		bool prev_header_completed = this->buffer_index >
			sizeof(this->header);
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
			if (this->header.topic_tag >= MSG_CT) {
				this->reset();
				return false;
			}

			/* does the msg_length make sense? */
			if (this->header.msg_length >= BUFFER_SZ) {
				this->reset();
				return false;
			}

			return false;
		} else if (prev_header_completed && packet_completed) {
			this->reset();
			return true;
		}
		return false;
	}

	void reset(void)
	{
		this->buffer_index = 0;
	}

	/* buffer incomming chars. */
	union {
		uint8_t buffer[BUFFER_SZ];
		/* convenient access to the buffer */
		PktHeader header;
	};

	uint8_t buffer_index;
};

template <size_t MSG_CT, size_t BUFFER_SZ>
class NodeHandle {
public:
	NodeHandle(char const *node_name)
		: name(node_name)
	{}

	/* retrieve the unique ID of the publisher */
	Publisher advertise(char const *topic)
	{
		return getTopicTag(topic);
	}

	void publish(Publisher pub, Msg *msg)
	{
		this->pout.pkt_start(PT_TOPIC, pub, msg->bytes());
		msg->serialize(&this->pout);
		this->pout.pkt_end();
	}

	void subscribe(char const *topic, RosCb *funct, Msg *msg)
	{
		uint8_t tag = getTopicTag(topic);
		this->cb_list[tag] = funct;
		this->msg_list[tag] = msg;
	}


	void spin(char c)
	{
		if (this->in_ctx.append(c)) {
			this->process_pkt();
		}
	}

	void spin(void)
	{
		int c;
		while ((c = byte_get()) != EOF) {
			this->spin(c);
		}
	}

private:
	string name;

	RosCb *cb_list[MSG_CT];
	Msg *msg_list[MSG_CT];

	void send_id()
	{
		MsgSz size = this->name.bytes();
		this->pout.pkt_start(PT_GETID, 0, size);
		this->name.serialize(&this->pout);
		this->pout.pkt_end();
	}

	void process_pkt()
	{
		switch(this->in_ctx.header.packet_type) {
		case PT_GETID:
			this->send_id();
			break;
		case PT_TOPIC:
			this->msg_list[this->in_ctx.header.topic_tag]->
				deserialize(this->in_ctx.buffer +
						sizeof(PktHeader));
			this->cb_list[this->in_ctx.header.topic_tag](this->
					msg_list[this->in_ctx.header.topic_tag]);
			break;
		case PT_SERVICE:
			break;
		}
	}

	/* given the character string of a topic, determines the numeric tag to
	 * place in a packet */
	/* char getTopicTag(char const *topic); */
#include "ros_get_topic_tag.h"

	InputCtx <MSG_CT, BUFFER_SZ> in_ctx;
	PacketOut pout;
};

} /* namespace ros */

#endif /* NODE_HANDLE_H_ */
