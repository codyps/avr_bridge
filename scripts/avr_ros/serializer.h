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

			static MsgSz deserialize(uint8_t *in, double& value) {
				// If a double is only 32-bits on this chip (a float), then convert
				// the 64-bit double being passed in to a 32-bit float
				if (sizeof(double) == 4) {
					union {
						double real;
						uint32_t raw;
						uint8_t base[sizeof(double)];
					} u;

					// Bit manipulation to convert 64-bit double to 32-bit float
					uint16_t expd = ((in[7] & 127) << 4) + ((in[6] & 240) >> 4);
					uint16_t expf = expd ? (expd - 1024) + 128 : 0;
					u.base[3] = (in[7] & 128) + (expf >> 1);
					u.base[2] = ((expf & 1) << 7) + ((in[6] & 15) << 3) + ((in[5] & 0xe0) >> 5);
					u.base[1] = ((in[5] & 0x1f) << 3) + ((in[4] & 0xe0) >> 5);
					u.base[0] = ((in[4] & 0x1f) << 3) + ((in[3] & 0xe0) >> 5);

					// IEEE 754 specifies "rounding half to even". To properly convert from a double precision
					// floating-point to a single-precision floating point, the rounding calculations need to
					// be applied.
					// Only consider rounding if the bit following the cut off (bit 35) is 1
					if ((in[3] & 0x10) >> 4 == 1) {
						// If any of the remaining bits starting at bit 36 are not 0, then round up
						if ((in[3] & 0xF) != 0 || in[2] != 0 || in[1] != 0 || in[0] != 0) {
							u.raw++;
						}
						// If all remaining bits following are 0 (bits starting at 36)
						// and the cut off bit (bit 34) is 1, then round up
						else if ((in[3] & 0x20) >> 5) {
							u.raw++;
						}
					}

					value = u.real;
				}
				// A double is 64-bits on this chip, deserializes bytes directly
				else {
					union {
						double real;
						uint32_t raw;
						uint8_t base[sizeof(double)];
					} u;

					for (uint8_t i = 0; i < sizeof(double); i++) {
						u.base[i] = in[i];
					}

					value = u.real;
				}

				return 8;
			}
	};

}

#endif

