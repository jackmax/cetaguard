#ifndef _AES_H_
#define _AES_H_
#include <stdint.h>
#include <pgmspace.h>

#define AES128_ROUNDS 10
#define AES128_SIZE 16
#define AES128_KEY_SIZE 16
#define AES128_SCHEDULE_SIZE ((AES128_ROUNDS + 1) * AES128_KEY_SIZE)

void AES_key_schedule(uint8_t schedule[AES128_SCHEDULE_SIZE], const uint8_t key[AES128_KEY_SIZE]);
void AES_encrypt(const uint8_t schedule[AES128_SCHEDULE_SIZE], uint8_t buf[AES128_SIZE]);
void AES_decrypt(const uint8_t schedule[AES128_SCHEDULE_SIZE], uint8_t buf[AES128_SIZE]);

#endif //_AES_H_
