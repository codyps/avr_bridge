/*
 * Msg.h
 *
 *  Created on: Oct 9, 2010
 *      Author: asher
 */

#ifndef MSG_H_
#define MSG_H_

#include <stdint.h>


class Msg {
public:
	Msg();
	Msg(uint8_t* data);
	virtual uint16_t serialize(uint8_t * outbuffer);
	virtual uint16_t deserialize(uint8_t * data);
	virtual uint16_t bytes();
	 ~Msg();
private:
	char * _buffer;
	int _length; //length of byte stream;

};



#endif /* MSG_H_ */
