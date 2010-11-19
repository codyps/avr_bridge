#include "RazorImu.h"
#include <stdio.h>
using namespace imu_9drazor ;

RazorImu::RazorImu() {} 


 RazorImu::~RazorImu(){
}

 RazorImu::RazorImu(uint8_t* data){
}

uint16_t  RazorImu::serialize(uint8_t* data){
int offset=0;
offset += this->angular_velocity.serialize(data + offset);
offset += this->linear_acceleration.serialize(data + offset);
*( (float *) (data + offset))=  this->roll; 
offset += 4;
*( (float *) (data + offset))=  this->pitch; 
offset += 4;
*( (float *) (data + offset))=  this->yaw; 
offset += 4;

 return offset;

}

uint16_t  RazorImu::deserialize(uint8_t* data){
int offset=0;
offset += this->angular_velocity.deserialize(data + offset);
offset += this->linear_acceleration.deserialize(data + offset);
this->roll = *( (float *) (data + offset) );
offset += 4;
this->pitch = *( (float *) (data + offset) );
offset += 4;
this->yaw = *( (float *) (data + offset) );
offset += 4;

 return offset;

}

uint16_t  RazorImu::bytes(){
 int msgSize=0;
msgSize += angular_velocity.bytes();
msgSize += linear_acceleration.bytes();
msgSize += sizeof(float);
msgSize += sizeof(float);
msgSize += sizeof(float);
return msgSize;

}
