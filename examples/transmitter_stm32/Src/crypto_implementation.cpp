#include "stm32l4xx_hal.h"
#include "crypto_primitives.h"
extern "C" {
#include "uECC.h"
#include "aes.h"
#include "sha2.h"
}
#include <stdlib.h>
#include <string.h>
#include <time.h>

extern RNG_HandleTypeDef hrng;
extern RTC_HandleTypeDef hrtc;

static int rng_for_microecc(uint8_t *dest, unsigned size) {
	return generate_random(dest, size);
}

void hash(const uint8_t* data, size_t length,  uint8_t hash_out[HASH_SIZE]){
	sha256(data, length, hash_out);
}

void encrypt(const uint8_t* in, uint8_t* out, const uint8_t* key){
	uint8_t rk[11*16];
	memcpy(rk, key, 16);
	AES_128_keyschedule(key, rk+16);
	AES_128_encrypt(rk, in, out);
}

void decrypt(const uint8_t* in, uint8_t* out, const uint8_t* key){
	uint8_t rk[11*16];
    memcpy(rk+160, key, 16);
    AES_128_keyschedule_dec(key, rk);
    AES_128_decrypt(rk, in, out);
}

bool generate_random(uint8_t* data, size_t length){
	size_t length4 = length - (length % 4);
	size_t i = 0;
	for (; i < length4; i+=4){
		if (HAL_RNG_GenerateRandomNumber(&hrng, (uint32_t*)(data + i)) != HAL_OK)
			return false;
	}
	if (length % 4 == 0)
		return true;
	uint32_t lastblock;
	if (HAL_RNG_GenerateRandomNumber(&hrng, &lastblock) != HAL_OK)
		return false;
	switch (length % 4){
	case 3:
		data[i++] = ((uint32_t*)&lastblock)[2];
	case 2:
		data[i++] = ((uint32_t*)&lastblock)[1];
	case 1:
		data[i++] = ((uint32_t*)&lastblock)[0];
	}
	return true;
}

static const struct uECC_Curve_t * curve = uECC_secp192r1();
	
bool generate_keys(uint8_t* pub_key, uint8_t* priv_key){
	uint8_t pub_key_uncompressed[2*CURVE_SIZE];
	uECC_set_rng(&rng_for_microecc);
	if (!uECC_make_key(pub_key_uncompressed, priv_key, curve)){
		return false;
	}
	else {
		uECC_compress(pub_key_uncompressed, pub_key, curve);
		return true;
	}
}

bool calculate_secret(uint8_t* pub, uint8_t* priv, uint8_t* secret){
	uint8_t pub_key_uncompressed[2*CURVE_SIZE];
	uECC_decompress(pub, pub_key_uncompressed, curve);
	return uECC_shared_secret(pub_key_uncompressed, priv, secret, curve);
}

uint32_t get_time(){
	RTC_DateTypeDef rtcDate;
	 RTC_TimeTypeDef rtcTime;
	 HAL_RTC_GetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
	 HAL_RTC_GetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);
	 uint8_t hh = rtcTime.Hours;
	 uint8_t mm = rtcTime.Minutes;
	 uint8_t ss = rtcTime.Seconds;
	 uint8_t d = rtcDate.Date;
	 uint8_t m = rtcDate.Month;
	 uint16_t y = rtcDate.Year;
	 uint16_t yr = (uint16_t)(y+2000-1900);
	 struct tm tim = {0};
	 tim.tm_year = yr;
	 tim.tm_mon = m - 1;
	 tim.tm_mday = d;
	 tim.tm_hour = hh;
	 tim.tm_min = mm;
	 tim.tm_sec = ss;
	 return mktime(&tim);
}

