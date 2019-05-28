#ifndef _AES_H_
#define _AES_H_

#include <stdint.h>
/*
 * AES on ARM implementation by Peter Schwabe and Ko Stoffelen. (https://github.com/Ko-/aes-armcortexm)
 * Released as part of the publication "All the AES You Need on Cortex-M3 and M4", published at SAC 2016.
 * The implementation is released into the public domain.
 */
void AES_128_keyschedule(const uint8_t *, uint8_t *);
void AES_128_keyschedule_dec(const uint8_t *, uint8_t *);
void AES_128_encrypt(const uint8_t *, const uint8_t *, uint8_t *);
void AES_128_decrypt(const uint8_t *, const uint8_t *, uint8_t *);
#endif //_AES_H_
