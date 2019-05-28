#include <osapi.h>
#include "crypto_primitives.h"
extern "C" {
#include "uECC.h"
#define ECB 1
#include "aes.h"
#include "sha2.h"
}
#include <stdlib.h>
#include <string.h>
#include <time.h>

static int RNG(uint8_t *dest, unsigned size) {
  return !os_get_random(dest, size);
}

void hash(const uint8_t* data, size_t length, uint8_t hash_out[HASH_SIZE]){
	sha256(data, length, hash_out);
}

void encrypt(const uint8_t* in, uint8_t* out, const uint8_t* key){
	uint8_t schedule[AES128_SCHEDULE_SIZE];
    if (in != out && (out >= in+ENCR_BLOCK_SIZE || in >= out+ENCR_BLOCK_SIZE)){ //if the memory areas don't overlap
    	memcpy(out, in, ENCR_BLOCK_SIZE);
    	AES_key_schedule(schedule, key);
		AES_encrypt(schedule, out);
	}
	else if (in == out){ //if the memory areas are the same
		AES_key_schedule(schedule, key);
		AES_encrypt(schedule, out);
	}
	else { //if the memory areas partially overlap
		uint8_t out_tmp[ENCR_BLOCK_SIZE];
		memcpy(out_tmp, in, ENCR_BLOCK_SIZE);
		AES_key_schedule(schedule, key);
		AES_encrypt(schedule, out_tmp);
		memcpy(out, out_tmp, ENCR_BLOCK_SIZE);
	}
}

void decrypt(const uint8_t* in, uint8_t* out, const uint8_t* key){
	uint8_t schedule[AES128_SCHEDULE_SIZE];
    if (in != out && (out >= in+ENCR_BLOCK_SIZE || in >= out+ENCR_BLOCK_SIZE)){ //if the memory areas don't overlap
    	memcpy(out, in, ENCR_BLOCK_SIZE);
    	AES_key_schedule(schedule, key);
		AES_decrypt(schedule, out);
	}
	else if (in == out){ //if the memory areas are the same
		AES_key_schedule(schedule, key);
		AES_decrypt(schedule, out);
	}
	else { //if the memory areas partially overlap
		uint8_t out_tmp[ENCR_BLOCK_SIZE];
		memcpy(out_tmp, in, ENCR_BLOCK_SIZE);
		AES_key_schedule(schedule, key);
		AES_decrypt(schedule, out_tmp);
		memcpy(out, out_tmp, ENCR_BLOCK_SIZE);
	}
}

bool generate_random(uint8_t* data, size_t length){
	return !os_get_random(data, length);
}

static const struct uECC_Curve_t * curve = uECC_secp192r1();
	
bool generate_keys(uint8_t* pub_key, uint8_t* priv_key){
	uint8_t pub_key_uncompressed[2*CURVE_SIZE];
	uECC_set_rng(&RNG);
	if (!uECC_make_key(pub_key_uncompressed, priv_key, curve)){
		return false;
	}
	else{
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
	return time(nullptr);
}

