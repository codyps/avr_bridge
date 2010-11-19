#include "Vec3.h"
#include <stdio.h>
using namespace imu_9drazor ;

Vec3::Vec3() {} 


 Vec3::~Vec3(){
}

 Vec3::Vec3(uint8_t* data){
}

uint16_t  Vec3::serialize(uint8_t* data){
int offset=0;
*( (float *) (data + offset))=  this->x; 
offset += 4;
*( (float *) (data + offset))=  this->y; 
offset += 4;
*( (float *) (data + offset))=  this->z; 
offset += 4;

 return offset;

}

uint16_t  Vec3::deserialize(uint8_t* data){
int offset=0;
this->x = *( (float *) (data + offset) );
offset += 4;
this->y = *( (float *) (data + offset) );
offset += 4;
this->z = *( (float *) (data + offset) );
offset += 4;

 return offset;

}

uint16_t  Vec3::bytes(){
 int msgSize=0;
msgSize += sizeof(float);
msgSize += sizeof(float);
msgSize += sizeof(float);
return msgSize;

}
