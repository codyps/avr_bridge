/*
 * vector.h
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

#ifndef ROS_SVECTOR_H_
#define ROS_SVECTOR_H_
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

namespace ros {

template<typename T> class static_vector {
public:
	static_vector(uint8_t *ndata, MsgSz len)
		: mem_size(len)
		, data(ndata)
	{}

	MsgSz size()
	{
		return length;
	}

	T& operator[](MsgSz i)
	{
		return data[i];
	}

	/* XXX:Broken: does not allow use of non-primitive messages.
	 * possible to fix via template specialization for primitive types.
	 */
	void serialize(PacketOut *p)
	{
		for(uint8_t i = 0; i < sizeof(length); i++) {
			p->pkt_send_byte((length >> i) & 0xff);
		}

		for(MsgSz i = 0; i < length; i++) {
			p->pkt_send_byte(data[i]);
		}
	}

	/* XXX:Broken: see serialize */
	MsgSz deserialize(uint8_t * buffer)
	{
		memcpy(&length, buffer, sizeof(length));
		buffer += sizeof(length);
		memcpy(data, buffer, this->bytes());
		return this->bytes();
	}

	/* size when serialized */
	/* XXX:Broken: see serialize */
	MsgSz bytes(void)
	{
		return length * sizeof(T) + sizeof(length);
	}

	void push_back(T item)
	{
		if (length > mem_size) {
			return;
		}
		
		data[length] = item;
		length++;
	}

	/* FIXME: potentially unsafe operation */
	T pop_back()
	{
		length--;
		return data[length + 1];
	}

	void set_mem(void *ndata, MsgSz nlen)
	{
		data = ndata;
		mem_size = nlen;
	}

private:
	T* data;
	MsgSz length;
	MsgSz mem_size;
};


}
#endif /* VECTOR_H_ */
