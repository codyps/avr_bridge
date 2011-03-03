/*
 * ros_string.cpp
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
 *
 *
 */

#include "avr_ros/string.h"
#include <string.h>
#include <stdlib.h>

using namespace ros;

void string::set_mem(void *ndata, MsgSz nlen)
{
	this->data = static_cast<char *>(ndata);
	this->mem_len = nlen;
}

string::string(char const *str)
{
	this->set_string(str);
}

string::string(char const *str, MsgSz len)
{
	this->set_nstring(str, len);
}

void string::set_nstring(char const *str, MsgSz l)
{
	this->str_len = l = (l > mem_len) ? mem_len : l;
	strncpy(this->data, str, l);
	this->data[l] = 0;
}

void string::set_string(char const *str)
{
	MsgSz l = strlen(str) + 1;
	set_nstring(str, l);
}

void string::serialize(PacketOut *p)
{
	MsgSz length = this->str_len;
	uint8_t i;
	for(i = 0; i < sizeof(length); i++) {
		p->pkt_send_byte((length >> i) & 0xff);
	}

	/* pad out to 4 bytes */
	for(; i < sizeof(uint32_t); i++) {
		p->pkt_send_bte(0);
	}

	for(MsgSz i = 0; i < length; i++) {
		p->pkt_send_byte(data[i]);
	}
}

MsgSz string::deserialize(uint8_t *buffer)
{
	uint32_t length;
	memcpy(&length, buffer, sizeof(length));
	buffer += sizeof(length);
	
	/* deal with the overflow quietly, just take as much as possible */
	str_len = (length > mem_len) ? mem_len : length;

	memcpy(data, buffer, str_len);
	data[str_len] = 0;

	return str_len + sizeof(length);
}

MsgSz string::bytes()
{
	return str_len + sizeof(uint32_t);
}

