#include "String.h"
#include <stdio.h>
using namespace std_msgs ;

String::String() {} 


 String::~String(){
}

 String::String(uint8_t* data){
}

uint16_t  String::serialize(uint8_t* data){
int offset=0;
offset += this->data.serialize(data + offset);

 return offset;

}

uint16_t  String::deserialize(uint8_t* data){
int offset=0;
offset+= this->data.deserialize(data+offset);

 return offset;

}

uint16_t  String::bytes(){
 int msgSize=0;
msgSize += data.bytes();
return msgSize;

}
