/*
 * Ros.h
 *
 *  Created on: Oct 9, 2010
 *      Author: asher
 */

#ifndef ROS_H_
#define ROS_H_
#include "WProgram.h"
#include "ros_string.h"
#include "Msg.h"


typedef void (*ros_cb)(Msg* msg);

struct packet_header{
		uint8_t packet_type;
		uint8_t topic_tag;
		uint16_t msg_length;
	};

typedef uint8_t Publisher;

class Ros {
public:
	Ros(char * node_name);
	Publisher advertise(char* topic);

	void publish(Publisher pub, Msg* msg);

	void subscribe(char* name, ros_cb funct, Msg* msg);
	void spin();

	void send(uint8_t* data, uint16_t length, char packet_type, char topicID); //handles actually sending the data

	ROS::string name;
	uint8_t outBuffer[300];

	~Ros();
private:
	char* topic_list[10];
	ros_cb cb_list[10];
	Msg * msgList[10];

	void getID();

	char getTopicTag(char * topic); //Used to get the topic tag for its packet
	//variables for handling incoming packets

	packet_header * header;
	int packet_data_left;
	uint8_t buffer[300];
	uint16_t buffer_index;
	enum packet_state{
		header_state , msg_data_state
	} com_state;

	void resetStateMachine();


};
extern Ros ros;



#endif /* ROS_H_ */
