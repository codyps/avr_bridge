/*
 * vector.h
 *
 *  Created on: Nov 1, 2010
 *      Author: asher
 */

#ifndef VECTOR_H_
#define VECTOR_H_
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

namespace ROS{

template<class T> class vector{
public:

	vector(){};
	vector(uint16_t size){fixed = 1;  setMaxLength(size); length=size;};
	uint16_t size(); //
	T& operator[] (int i) { return data[i];}
	uint16_t serialize(uint8_t * buffer);
	uint16_t deserialize(uint8_t * buffer);
	uint16_t bytes();// output size to store in bytes

	void push_back(T item){
		if (length <maxLength) {
		data[length] = item;
		length++;
				}
	}

	T pop_back(){length--; return data[length+1];};

	~vector(){free(data);}
	bool fixed;

	void setMaxLength(uint16_t L){ maxLength = L;
								data = (T*)malloc( L* sizeof(T));};

private:
	T* data;
	uint16_t length;
	uint16_t maxLength;
	void setSize(uint16_t size){ length = size;};
	uint16_t serializeFixed(uint8_t * buffer);
	uint16_t serializeVariable(uint8_t* buffer);
	uint16_t deserializeFixed(uint8_t *buffer);
	uint16_t deserializeVariable(uint8_t * buffer);
};



template<class T>
uint16_t vector<T>::size(){
	return length;
}
template<class T>
uint16_t vector<T>::bytes(){
	return length*sizeof(T);
}

template<class T>
uint16_t vector<T>::serializeFixed(uint8_t * buffer){
	memcpy(buffer,data, this->bytes());
	return this->bytes();
}


template<class T>
uint16_t vector<T>::serializeVariable(uint8_t * buffer){
	memcpy(buffer, &length, 2);
	serializeFixed(buffer+4);
	return this->bytes()+4;
}

template<class T>
uint16_t vector<T>::deserializeFixed(uint8_t * buffer){
	memcpy(data,buffer, this->bytes());
	return this->bytes();
}
template<class T>
uint16_t vector<T>::deserializeVariable(uint8_t * buffer){
	uint32_t size;
	memcpy(&size, buffer, 4);
	setSize(size);
	deserializeFixed(buffer+4);
	return this->bytes()+4;
}


template<class T>
uint16_t vector<T>::serialize(uint8_t * buffer){
	if (fixed) return serializeFixed(buffer);
	else return this->serializeVariable(buffer);
}

template<class T>
uint16_t vector<T>::deserialize(uint8_t* buffer){
	if (fixed) return deserializeFixed(buffer);
	else return deserializeVariable(buffer);
}

}
#endif /* VECTOR_H_ */
