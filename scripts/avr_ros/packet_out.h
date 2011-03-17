#ifndef PACKET_OUT_H_
#define PACKET_OUT_H_

#include <stdio.h>
#include <avr_ros/types.h>
#include <avr_ros/byte_io.h>

namespace ros {
	enum PktType {
		PT_TOPIC = 0,
		PT_SERVICE = 1,
		PT_GETID = 0xff
	};

	struct PktHeader {
		uint8_t packet_type;
		uint8_t topic_tag;
		uint16_t msg_length;
	};

	class PacketOut {
	public:
		void pkt_start(enum PktType pkt_type, uint8_t topic,
				MsgSz data_len)
		{
			PktHeader head = {
				pkt_type,
				topic,
				data_len
			};

			uint8_t *hp = (typeof(hp))&head;

			uint8_t i;
			for (i = 0; i < sizeof(head); i++) {
				byte_put(hp[i]);
			}
		}

		void pkt_end(void)
		{
			/* nop */
		}

		void pkt_send_byte(uint8_t c)
		{
			byte_put(c);
		}
	};
}

#endif
