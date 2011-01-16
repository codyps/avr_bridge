/*
 * Ros.h
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

#ifndef ROS_H_
#define ROS_H_
#include "WProgram.h"
#include "ros_string.h"
#include "Msg.h"

#include <stdint.h>

#define __depricated __attribute__((depricated))

#ifndef UINT8_MAX
#define UINT8_MAX 0xff
#endif

#define ROS_BUFFER_SIZE (UINT8_MAX + 1)
typedef void (*ros_cb)(Msg *msg);

struct PktHeader {
		uint8_t packet_type;
#define PT_TOPIC	(0)
#define PT_SERVICE	(1)
#define PT_GETID	(0xff)
		uint8_t topic_tag;
		uint16_t msg_length;
};

typedef uint8_t Publisher;

struct RosInputCtx {
	RosInputCtx(uint8_t _topic_tag_max);
	bool append(char c);
	void reset(void) {
		buffer_index = 0;
	}

	/* only used for a sanity check */
	uint8_t topic_tag_max;


	/* buffer incomming chars. */
	union {
		uint8_t buffer[ROS_BUFFER_SIZE];
		/* convenient access to the buffer */
		PktHeader header;
	};

	uint8_t buffer_index;
};

class Ros {
public:
	Ros(char const *node_name, uint8_t num_of_msg_types);

	Publisher advertise(char const *topic);
	void publish(Publisher pub, Msg *msg);
	void subscribe(char const *name, ros_cb funct, Msg *msg);

	void spin(char c);

	__depricated
	void spin();

	/* XXX: the types are possibly wrong (or at least misleading), and the
	 * argument ordering is not natural. The name `send` is also unclear. */
	void send(uint8_t const *data, uint16_t length,
			char packet_type, char topicID);


	/* XXX: these do not exsist in Ros.cpp, should they be removed? */
	void init_node();
	void initCommunication();

	~Ros();
private:
	ROS::string name;

	ros_cb cb_list[10];
	Msg *msg_list[10];
	uint8_t outBuffer[UINT8_MAX + 1];

	void getID();
	void process_pkt();

	/* given the character string of a topic, determines the numeric tag to
	 * place in a packet */
	char getTopicTag(char const *topic);

	RosInputCtx in_ctx;

	/* XXX: these two fields exsists for the sole purpose of compatability
	 * with the exsisting python code. */
	uint8_t *buffer;
	uint8_t buffer_index;
};

#endif /* ROS_H_ */
