/*
 * string.h
 *
 *  Created on: Nov 1, 2010
 *      Author: asher
 */

#ifndef STRING_H_
#define STRING_H_
#include <stdint.h>

//this is only meant for rosdatatypes

namespace ROS{
class string{
public:
	string(uint16_t maxLength);
	string(char * str);
	string();
	void setString(char* str);
	uint16_t bytes();
	uint16_t serialize(uint8_t* buffer);
	uint16_t deserialize(uint8_t* buffer);
	char operator[](int i){ return data[i];};
	void setMaxLength( uint16_t maxLength);
	char* getRawString(){return data;};

private:
	char* data;
	uint16_t maxlength;
};

}
#endif /* STRING_H_ */
