#ifndef CETAGUARD_H
#define CETAGUARD_H

#include "crypto_primitives.h"
#include <vector>
#include <stdint.h>
#include <stddef.h>

//configurable parameters
#define ROLLING_CODE_TOLERANCE 256
#define TOLERANCE_PPM 40.0f

#define P_SECRET_SIZE 16
#define OUT_HASH_SIZE 24

/*TODO:
make functions that export transmitter/receiver data for storage
make constructors that re-create transmitter/receiver from stored data
possibility: separate receiver into 2 classes: one with only global key, one with global and separate keys
*/

#define CETAGUARD_MAGIC_0 0x01
#define CETAGUARD_MAGIC_1 0x02
#define CETAGUARD_MAGIC_2 0x03
#define CETAGUARD_VERSION 0x1
#define CETAGUARD_TYPE_BUTTON 0x1
#define CETAGUARD_TYPE_PAIRING_NS 0x2
#define CETAGUARD_TYPE_PAIRING_S 0x3
#define CETAGUARD_MSG_MAX_SIZE 64

typedef int Cetaguard_index;
typedef enum
{
    CETAGUARD_OK = 0,
    CETAGUARD_KEY_GEN_FAILED,
    CETAGUARD_SECRET_CALC_FAILED,
    CETAGUARD_RANDOM_FAILED,
    CETAGUARD_IMPORT_FAILED,
    CETAGUARD_INDEX_OOBOUNDS,
    CETAGUARD_NOT_PAIRING,
    CETAGUARD_MSG_TOO_SHORT,
    CETAGUARD_MSG_BAD_PREAMBLE,
    CETAGUARD_MSG_VERSION_UNSUPPORTED,
    CETAGUARD_MSG_UNKNOWN_TYPE,
    CETAGUARD_MSG_BAD_MAC,
    CETAGUARD_MSG_BAD_COUNTER,
    CETAGUARD_MSG_TXR_NOT_PENDING,
    CETAGUARD_MSG_TXR_NOT_PAIRED,
    CETAGUARD_MSG_TXR_ALREADY_PAIRED,
} Cetaguard_status;

typedef enum
{
    CETAGUARD_CLOCK_DISABLE = 0,  //Disables checking of clock congruence
    CETAGUARD_CLOCK_INTERNAL = 1,
    CETAGUARD_CLOCK_LOCAL = 2,
    CETAGUARD_CLOCK_REMOTE = 3,
} Cetaguard_clock_source;

typedef struct
{
    uint32_t transmitter_id; //Transmitter ID used for this receiver
    uint16_t reset_ctr;
    uint16_t rolling_ctr;
    uint8_t pub_key[PUB_KEY_SIZE];
    uint8_t priv_key[PRIV_KEY_SIZE];
    uint8_t receiver_pub_key[PUB_KEY_SIZE];
    uint8_t transmission_key[ENCR_KEY_SIZE];
} Cetaguard_paired_receiver;

typedef struct
{
    uint32_t transmitter_id;
    uint32_t secret_hash;
    //TODO: maybe add time of adding it to list so we can timeout?
    uint8_t pairing_secret[P_SECRET_SIZE];
    uint8_t receiver_pub_key[PUB_KEY_SIZE];
    uint8_t pub_key[PUB_KEY_SIZE];
    uint8_t priv_key[PRIV_KEY_SIZE];
    uint8_t pairing_secret_used :1;
} Cetaguard_pending_receiver;

typedef struct
{
    uint32_t transmitter_id;
    uint32_t last_time;
    uint32_t last_time_recv;
    uint16_t reset_ctr;
    uint16_t rolling_ctr;
    uint8_t pub_key[PUB_KEY_SIZE];
    uint8_t priv_key[PRIV_KEY_SIZE];
    uint8_t transmission_key[ENCR_KEY_SIZE];
    uint8_t data_initialized :1;
} Cetaguard_paired_transmitter;

typedef struct
{
    uint32_t transmitter_id;
    uint32_t secret_hash;
    //TODO: maybe add time of adding it to list so we can timeout?
    uint8_t pairing_secret[SECRET_SIZE];
    uint8_t pub_key[PUB_KEY_SIZE];
    uint8_t priv_key[PRIV_KEY_SIZE];
} Cetaguard_pending_transmitter;

typedef struct
{
   uint32_t buttons;
   uint8_t pattern;
   uint8_t battery;
} Cetaguard_msg_contents;

class Cetaguard_message_interpreter
{
public:
	Cetaguard_message_interpreter(){
	}
    virtual void button_msg(Cetaguard_msg_contents contents, Cetaguard_index idx){
	}
    virtual void pairing_without_secret_msg(Cetaguard_index idx){
    }
    virtual void pairing_with_secret_msg(Cetaguard_index idx){
	}
};

class Cetaguard_exporter_importer
{
public:
    Cetaguard_exporter_importer(){
    }
    virtual bool read_byte(uint8_t* byte_out){
        return false;   
    }
    virtual bool write_byte(uint8_t byte){
        return false;   
    }
    virtual size_t read_bytes(uint8_t* bytes_out, size_t max_out){
        size_t total_read = 0;
        while (total_read < max_out){
            if (read_byte(&(bytes_out[total_read]))){
                total_read++;
            }
            else {
                break;
            }
        }
        return total_read;
    }
    virtual size_t write_bytes(uint8_t* bytes, size_t length){
        size_t i;
        for (i = 0; i<length; i++){
            if (!write_byte(bytes[i])){
                break;
            }
        }
        return i;
    }
};

class Cetaguard_transmitter
{
public:
	Cetaguard_transmitter();
    Cetaguard_transmitter(Cetaguard_exporter_importer* e);
    Cetaguard_status add_receiver(const uint8_t* recv_pub_key);
    Cetaguard_status add_receiver(const uint8_t* recv_pub_key, const uint8_t* pairing_secret);
    Cetaguard_status prepare_pairing_msg(Cetaguard_index idx, uint8_t* msg_out, size_t* size_out);
    Cetaguard_status finish_pairing(Cetaguard_index idx);
    Cetaguard_status prepare_button_msg(Cetaguard_index idx, const Cetaguard_msg_contents* contents, uint8_t* msg, size_t* size);
    Cetaguard_status remove_receiver(Cetaguard_index idx);
    Cetaguard_status remove_pending(Cetaguard_index idx);
    void increment_reset_counters();
    inline Cetaguard_paired_receiver* get_recv_ptr(Cetaguard_index idx){
        if (idx < 0 || idx >= pending.size()){
            return NULL;
        }
        return &(receivers[idx]);
    }

    inline Cetaguard_pending_receiver* get_pending_ptr(Cetaguard_index idx){
        if (idx < 0 || idx >= pending.size()){
            return NULL;
        }
        return &(pending[idx]);
    }
    bool export_state(Cetaguard_exporter_importer* e);
    std::vector<Cetaguard_paired_receiver> receivers;
private:

    std::vector<Cetaguard_pending_receiver> pending;
};

class Cetaguard_receiver
{
public:
	Cetaguard_receiver(Cetaguard_message_interpreter* interpreter_);
    Cetaguard_receiver(Cetaguard_message_interpreter* interpreter_, Cetaguard_exporter_importer* e);
    Cetaguard_status add_pending_transmitter(uint32_t* id_out);
    Cetaguard_status interpret_message(uint8_t* msg_in, size_t length, uint32_t recv_time); //WARNING! function may clobber msg_in
    inline Cetaguard_status interpret_message(uint8_t* msg_in, size_t length){ //WARNING! function may clobber msg_in
        return interpret_message(msg_in, length, get_time());
    }
    Cetaguard_status remove_transmitter(Cetaguard_index idx);
    Cetaguard_index find_transmitter(uint32_t transmitter_id);
    Cetaguard_index find_pending(uint32_t secret_hash);
    Cetaguard_status remove_pending(Cetaguard_index idx);

    inline void set_pairing_mode(bool on){
        in_pairing_mode = on;
    }

    inline bool get_pairing_mode(){
        return in_pairing_mode;
    }

    inline void set_clock_source(Cetaguard_clock_source s){
    	clock_source = s;
	}

	inline Cetaguard_clock_source get_clock_source(){
		return (Cetaguard_clock_source)clock_source;
	}

    inline void time_was_synchronized(){
        time_at_last_sync = get_time();
    }

    inline const uint8_t* get_pairing_secret(Cetaguard_index idx){
        if (idx < 0 || idx >= pending.size()){
            return NULL;
        }
        return pending[idx].pairing_secret;
    }

    inline const uint8_t* get_public_key_pending(Cetaguard_index idx){
        if (idx < 0 || idx >= pending.size()){
            return NULL;
        }
        return pending[idx].pub_key;
    }

    inline const uint8_t* get_public_key_static(){
        return static_pub_key;
    }

    bool export_state(Cetaguard_exporter_importer* e);
    std::vector<Cetaguard_paired_transmitter> transmitters;
private:
	Cetaguard_message_interpreter* interpreter;

    std::vector<Cetaguard_pending_transmitter> pending;
    uint8_t static_priv_key[PRIV_KEY_SIZE];
    uint8_t static_pub_key[PUB_KEY_SIZE];
    uint32_t time_at_last_sync;
    uint8_t in_pairing_mode :1;
    uint8_t clock_source    :2;
    bool is_time_congruent(uint32_t last_time, uint32_t msg_time, uint32_t last_time_recv, uint32_t recv_time);
};

#endif /* CETAGUARD_H */
