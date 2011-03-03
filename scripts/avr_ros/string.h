/*
 * string.h
 *
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Adam Stambler
 * Copyright (c) 2011, Cody Schafer <cpschafer@gmail.com>
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
 */

#ifndef ROS_STRING_H_
#define ROS_STRING_H_
#include <stdint.h>
#include <avr_ros/ros_types.h>
#include <avr_ros/packet_out.h>

//this is only meant for rosdatatypes

namespace ros {

	class string {
	public:
		string(){}

		string(char const *str);
		void set_string(char const *str);

		/* note that length includes null terminator */
		string(char const *str, MsgSz len);
		void set_nstring(char const *str, MsgSz l);
		
		char operator[](MsgSz i)
		{
			return data[i];
		}
		
		char *c_str()
		{
			return data;
		}

		MsgSz bytes();
		void serialize(PacketOut *p);
		MsgSz deserialize(uint8_t *buffer);

		void set_mem(void *data, MsgSz mem_len);
		void set_len(MsgSz l)
		{
			str_len = l;
		}

		MsgSz size(void)
		{
			return mem_len;
		}
	private:
		char *data;
		MsgSz mem_len;
		MsgSz str_len;
	};

} /* namespace ros */
#endif /* STRING_H_ */
