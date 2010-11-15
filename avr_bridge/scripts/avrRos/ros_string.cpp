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

}

string::string(uint16_t maxLength){
	this->setMaxLength(maxLength);
}

void string::setMaxLength( uint16_t maxLength){
	data = (char*) malloc(maxLength+1);
	this->maxlength = maxLength;
}

string::string(char * str){
	this->setString(str);
}

void string::setString(char * str){
	this->setMaxLength( strlen(str));
	strcpy(data, str);
}

uint16_t string::serialize(uint8_t * buffer){
	uint32_t length = strlen(data);
	memcpy(buffer, &length, 4);
	memcpy(buffer+4, data, length);
	return length+4;
}

uint16_t string::deserialize(uint8_t* buffer){
	uint32_t length;
	if (length> maxlength){
		free(data);
		data = (char*) malloc(length+1);
	}
	memcpy(&length, buffer,4);
	memcpy(data, buffer+4, length);
	data[length] =0; //add null terminating charater
	return length+4;
}

uint16_t string::bytes(){
	uint32_t length = strlen(data);
	return length+4;
}

