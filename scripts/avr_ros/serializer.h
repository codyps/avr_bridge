#ifndef SERIALIZER_H_
#define SERIALIZER_H_

#include <stdint.h>
#include <avr_ros/types.h>
#include <avr_ros/packet_out.h>


namespace ros {

	class Serializer {
		public:

			/**
			 * Writes value to byte stream.
			 *
			 * @param  out - packet handler to write to
			 * @param  value - the data to write to the stream
			 */
			template <typename TYPE>
			static void serialize(ros::PacketOut *out, TYPE value) {
				union {
					TYPE real;
					uint8_t base[sizeof(TYPE)];
				} u;
				u.real = value;

				// Outputs each byte to the byte stream
				for (uint8_t i = 0; i < sizeof(TYPE); i++) {
					out->pkt_send_byte(u.base[i]);
				}
			}

			/**
			 * Reads value from byte stream.
			 *
			 * @param  in - the byte array to read from
			 * @param  value - sets the read data into this value
			 * @return the byte stream offset (size of value in bytes)
			 */
			template <typename TYPE>
			static MsgSz deserialize(uint8_t *in, TYPE& value) {
				union {
					TYPE real;
					uint8_t base[sizeof(TYPE)];
				} u;

				// Reads the value, byte by byte from the stream
				for (uint8_t i = 0; i < sizeof(TYPE); i++) {
					u.base[i] = in[i];
				}
				value = u.real;

				return sizeof(TYPE);
			}

	};

}

#endif

