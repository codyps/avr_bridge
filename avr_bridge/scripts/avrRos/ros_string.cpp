/*
 * ros_string.cpp
 *
 *  Created on: Nov 1, 2010
 *      Author: asher
 */

#include "ros_string.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

using namespace ROS;


string::string(){
	maxlength = 0;
}

string::string(uint16_t maxLength){
	maxLength=0;
	this->setMaxLength(maxLength);
}

void string::setMaxLength( uint16_t maxLength){
	if (this->maxlength <=0){
		data = (char*) malloc(maxLength+1);
		this->maxlength = maxLength;
		data[0]=0;
	}
}

string::string(char * str){
	this->setString(str);
}

void string::setString(char * str){
	int l = strlen(str);

	if (maxlength<=0) setMaxLength(l);

	l = (l > maxlength) ? maxlength : l;

	strncpy(this->data,str,l);
	this->data[l]=0;

}

uint16_t string::serialize(uint8_t * buffer){
	uint32_t length = strlen(data);
	memcpy(buffer, &length, 4);
	memcpy(buffer+4, data, length);
	return length+4;
}

uint16_t string::deserialize(uint8_t* buffer){
	uint32_t length;
	memcpy(&length, buffer,4);
	//deal with the overflow quietly, just take as much as possible
	if (length > maxlength){
		memcpy(data, buffer+4, maxlength);
		data[maxlength] =0;
	}
	else{
		memcpy(data, buffer+4, length);
		data[length] =0;
	}



	 //add null terminating charater
	return length+4;
}

uint16_t string::bytes(){
	uint32_t length = strlen(data);
	return length+4;
}

