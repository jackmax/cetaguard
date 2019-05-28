#ifndef CRYPTO_PRIMITIVES_H
#define CRYPTO_PRIMITIVES_H

#include <stdint.h>
#include <stddef.h>


#define HASH_SIZE 32
#define HASH_BLOCK_SIZE 64
/**
Hash function primitive.
@param data Data to be hashed
@param length Length in bytes of \var data
@param hash Hash of \var data of size \var HASH_SIZE
*/
void hash(const uint8_t* data, size_t length, uint8_t hash_out[HASH_SIZE]);

#define ENCR_BLOCK_SIZE 16
#define ENCR_KEY_SIZE 16

/**
Block encryption function primitive. Must allow \var in and \var out to overlap.
@param[in] in Data to be encrypted.
@param[out] out Encrypted data.
@param[in] key Encryption key to use
*/
void encrypt(const uint8_t* in, uint8_t* out, const uint8_t* key);
void decrypt(const uint8_t* in, uint8_t* out, const uint8_t* key);

/**
Random number generation primitive.
@param[out] data Generated random data.
@param[in] length Number of bytes requested.
@return returns 1 on success, 0 on failure
*/
bool generate_random(uint8_t* data, size_t length);

#define CURVE_SIZE 24
//+4 to align everything to 32 bits
#define PUB_KEY_SIZE (CURVE_SIZE+4)
#define PRIV_KEY_SIZE CURVE_SIZE
#define SECRET_SIZE CURVE_SIZE

/**
Public/private key pair generation primitive. Generates a random pair of keys.
@param[out] key Generated public key.
@return returns 1 on success, 0 on failure
*/
bool generate_keys(uint8_t* pub_key, uint8_t* priv_key);

/**
Common secret calculation primitive.
@param[out] key Generated private key.
@return returns 1 on success, 0 on failure
*/
bool calculate_secret(uint8_t* pub, uint8_t* priv, uint8_t* secret);

uint32_t get_time();

#endif /* CRYPTO_PRIMITIVES_H */
