#ifndef PACKET_OUT_H_
#define PACKET_OUT_H_

#include <stdio.h>
#include <avr_ros/types.h>

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
	private:
		FILE *byte_io;
	public:
		PacketOut(FILE *byte_io_)
			: byte_io(byte_io_)
		{}
	public:
		void pkt_start(enum PktType pkt_type, uint8_t topic,
				MsgSz data_len)
		{
			PktHeader head = {
				pkt_type,
				topic,
				data_len
			};

			fwrite(&head, sizeof(head), 1, byte_io);
		}

		void pkt_end(void)
		{
			/* nop */
		}

		void pkt_send_byte(uint8_t c)
		{
			putc(c, byte_io);
		}

		uint8_t pkt_recv_byte(void)
		{
			return getc(byte_io);
		}
	};
}

#endif
