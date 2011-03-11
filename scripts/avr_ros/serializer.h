#ifndef SERIALIZER_H_
#define SERIALIZER_H_

#include <stdint.h>
#include <avr_ros/types.h>
#include <avr_ros/packet_out.h>


namespace ros {

	class Serializer {
		private:
			/**
			 * Reverses the byte order to handle little and big endian
			 * conversions.
			 *
			 * @param buffer - byte array to reverse
			 * @param size - length of array to reverse
			 */
			static void reverse_bytes(uint8_t *bytes, uint16_t size) {
				for (uint16_t i = 0; i < size / 2; i++ ) {
					uint8_t swap_byte = bytes[i];
					bytes[i] = bytes[size - i - 1];
					bytes[size - i - 1] = swap_byte;
				}
			}

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
					uint8_t bytes[sizeof(TYPE)];
				} u;
				u.real = value;

				// Outputs each byte to the byte stream
				for (uint8_t i = 0; i < sizeof(TYPE); i++) {
					out->pkt_send_byte(u.bytes[i]);
				}
			}

			/**
			 * Writes doubles to byte stream. If the double is actually a
			 * single precision float (32-bit) as is common on 8-bit AVRs, then
			 * converts to a double precision float (64-bit).
			 *
			 * @param  out - packet handler to write to
			 * @param  value - the data to write to the stream
			 */
			static void serialize(ros::PacketOut *out, double value) {
				// If a double is treated as a single precision floating point
				// value (a 32-bit float), then converts to a double precision
				// floating point value (64-bit double)
				if (sizeof(double) == 4) {
					// Calculates the exponent and significand of the 32-bit
					// float
					union {
						double real;
						uint8_t bytes[sizeof(double)] ;
					} float32;
					float32.real = value;
					// 8-bit AVRs are little endian, reverse bytes for
					// calculations
					Serializer::reverse_bytes(float32.bytes, sizeof(double));

					uint32_t exponent    = ((float32.bytes[0] & 0x7F) << 1)
						| ((float32.bytes[1] & 0x80) >> 7);
					uint32_t significand = ((((uint32_t)(float32.bytes[1] & 0x7F)) << 16)
						| (float32.bytes[2] << 8)
						| (float32.bytes[3]));

					// Sets the sign bit, exponent, and significand of the
					// 64-bit double
					union {
						uint64_t real;
						uint8_t bytes[8];
					} double64;
					double64.real = 0;

					// Sets the sign bit
					double64.bytes[0] = float32.bytes[0] & 0x80;

					// Sets the exponent
					if (exponent == 0) {
						// +/- 0
						if (significand  == 0) {
							// Nothing left to do
						}
						// Sub-normal number
						else {
							// Only set the significand
						}
					}
					// +/-INF and NaN
					else if (exponent == 0xFF) {
						double64.bytes[0] |= 0x7F;
						double64.bytes[1]  = 0xF0;
					}
					// Normal number
					else {
						int16_t int_exp = exponent;
						// IEEE 754 single precision exponent bias
						int_exp -= 127;
						// IEEE 754 double precision exponent bias
						int_exp += 1023;
						double64.bytes[0] |= (int_exp & 0x7F0) >> 4;
						double64.bytes[1]  = (int_exp & 0x00F) << 4;
					}

					// Sets the significand, most significant bits first
					if (significand != 0) {
						double64.bytes[1] |= (float32.bytes[1] & 0x78) >> 3;
						double64.bytes[2]  = (((float32.bytes[1] & 0x07) << 5)
							| ((float32.bytes[2] & 0xF8) >> 3));
						double64.bytes[3]  = (((float32.bytes[2] & 0x07) << 5)
							| ((float32.bytes[3] & 0xF8) >> 3));
						double64.bytes[4]  = ((float32.bytes[3] & 0x07) << 5);
					}

					// Outputs each byte to the byte stream.
					// As the bytes were reversed earlier for little endianess,
					// re-reverses when outputting to stream
					Serializer::reverse_bytes(double64.bytes, 8);
					for (uint8_t i = 0; i < 8; i++) {
						out->pkt_send_byte(double64.bytes[i]);
					}
				}
				// A double is 64-bits on this chip, serializes bytes directly
				else {
					union {
						double real;
						uint8_t bytes[sizeof(double)];
					} u;
					u.real = value;

					// Outputs each byte to the byte stream
					for (uint8_t i = 0; i < sizeof(double); i++) {
						out->pkt_send_byte(u.bytes[i]);
					}
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
					uint8_t bytes[sizeof(TYPE)];
				} u;

				// Reads the value, byte by byte from the stream
				for (uint8_t i = 0; i < sizeof(TYPE); i++) {
					u.bytes[i] = in[i];
				}
				value = u.real;

				return sizeof(TYPE);
			}

			static MsgSz deserialize(uint8_t *in, double& value) {
				// If a double is only 32-bits on this chip (a float), then
				// convert the 64-bit double being passed in to a 32-bit float
				if (sizeof(double) == 4) {
					union {
						double real;
						uint32_t raw;
						uint8_t bytes[sizeof(double)];
					} u;

					// Bit manipulation to convert 64-bit double to 32-bit float
					uint16_t expd = ((in[7] & 127) << 4) + ((in[6] & 240) >> 4);
					uint16_t expf = expd ? (expd - 1024) + 128 : 0;
					u.bytes[3] = (in[7] & 128) + (expf >> 1);
					u.bytes[2] = ((expf & 1) << 7) + ((in[6] & 15) << 3) + ((in[5] & 0xe0) >> 5);
					u.bytes[1] = ((in[5] & 0x1f) << 3) + ((in[4] & 0xe0) >> 5);
					u.bytes[0] = ((in[4] & 0x1f) << 3) + ((in[3] & 0xe0) >> 5);

					// IEEE 754 specifies "rounding half to even". To properly
					// convert from a double precision floating-point to a
					// single-precision floating point, the rounding
					// calculations need to be applied.
					// Only consider rounding if the bit following the cut off
					// (bit 35) is 1.
					if ((in[3] & 0x10) >> 4 == 1) {
						// If any of the remaining bits starting at bit 36 are
						// not 0, then round up
						if ((in[3] & 0xF) != 0 || in[2] != 0 || in[1] != 0 || in[0] != 0) {
							u.raw++;
						}
						// If all remaining bits following are 0 (bits starting
						// at 36) and the cut off bit (bit 34) is 1, then round
						// up
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
						uint8_t bytes[sizeof(double)];
					} u;

					for (uint8_t i = 0; i < sizeof(double); i++) {
						u.bytes[i] = in[i];
					}

					value = u.real;
				}

				return 8;
			}
	};

}

#endif
