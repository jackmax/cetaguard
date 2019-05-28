#ifndef CRC32_H
#define CRC32_H
#include <stdint.h>
#include <pgmspace.h>

/*
 * Based on crc32.c from util-linux v2.17 - Public Domain
 * Adjusted to make use of ESP8266 PROGMEM for the lookup table
 * The function used to generate the lookup table is included as crc32init(uint32_t*)
 */

#define CRC_INIT 0xFFFFFFFF
#define CRC_LE_BITS 8
#define CRCPOLY_LE 0xedb88320

void crc32_init(uint32_t* crc_table);
/*
 * Calculates the CRC32 checksum of given data in little-endian format using the polynomial used for Ethernet.
 * @param crc - CRC checksum of the previous of data. If none, use CRC_INIT
 * @param p - pointer to data to be chemsummed
 * @param len - length of data to be checksummed
 */
uint32_t crc32(uint32_t crc, unsigned char const *p, size_t len);

#endif /* CRC32_H */
