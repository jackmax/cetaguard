#include "cetaguard.h"
#include <string.h>
#include <math.h>
#include <Arduino.h>

typedef struct
{
	uint8_t magic[3];
    uint8_t msg_type :4;
    uint8_t version :4;
} Cetaguard_preamble;

typedef struct
{	
	Cetaguard_preamble pre;
    uint32_t transmitter_id;
    uint8_t pub_key[PUB_KEY_SIZE]; //TODO: make sure it's aligned to 4 bytes on all systems
    uint8_t hmac[OUT_HASH_SIZE];
} Cetaguard_pairing_message;

typedef struct
{
    Cetaguard_preamble pre;
    uint32_t transmitter_id;
    uint64_t init_value;
    uint8_t hmac[OUT_HASH_SIZE];
    uint32_t time_value;
    uint32_t btn_press_value;
    uint16_t reset_ctr;
    uint16_t rolling_ctr;
    uint8_t battery;
    uint8_t pattern;
    uint8_t padding_1;
    uint8_t padding_2;
} Cetaguard_button_message;

extern volatile uint32_t debug_data[64];
extern volatile uint32_t debug_data_ctr;

/*===== RECEIVER CODE =====*/
Cetaguard_receiver::Cetaguard_receiver(Cetaguard_message_interpreter* interpreter_){
	interpreter = interpreter_;
	if (!generate_keys(static_pub_key, static_priv_key)){
		//throw CETAGUARD_KEY_GEN_FAILED;
	}
	
}

Cetaguard_status Cetaguard_receiver::add_pending_transmitter(uint32_t* id_out){
	Cetaguard_pending_transmitter p;
	debug_data[0] = ESP.getCycleCount();
	if (!generate_keys(p.pub_key, p.priv_key)){
		return CETAGUARD_KEY_GEN_FAILED;
	}
	debug_data[1] = ESP.getCycleCount();
	if (!generate_random(p.pairing_secret, P_SECRET_SIZE)){
		return CETAGUARD_RANDOM_FAILED;
	}
	debug_data[2] = ESP.getCycleCount();
	uint8_t secret_hash[HASH_SIZE];
	hash(p.pairing_secret, P_SECRET_SIZE, secret_hash);
	debug_data[3] = ESP.getCycleCount();
	//TODO: one or both of the values below can duplicate existing ones!!!
	p.secret_hash = *(uint32_t*)secret_hash;
	p.transmitter_id = *(uint32_t*)(secret_hash + HASH_SIZE - sizeof(uint32_t));
	pending.push_back(p);
	*id_out = p.secret_hash;
	return CETAGUARD_OK;
}

const uint8_t* Cetaguard_receiver::get_pairing_secret(Cetaguard_index idx){
	if (idx < 0 || idx >= pending.size()){
		return NULL;
	}
	return pending[idx].pairing_secret;
}
const uint8_t* Cetaguard_receiver::get_public_key(Cetaguard_index idx){
	if (idx < 0 || idx >= pending.size()){
		return NULL;
	}
	return pending[idx].pub_key;
}

//Determines whether the transmitter has been reset since last message. 
//If last_reset_ctr already has max value of 0xFFFF always returns false
static inline bool has_been_reset(uint16_t last_reset_ctr, uint16_t current_reset_ctr){
	return (last_reset_ctr != 0xFFFF) && current_reset_ctr == last_reset_ctr + 1;
}

static inline bool is_counter_congruent(uint16_t last_rolling_ctr, uint16_t current_rolling_ctr){
	if (last_rolling_ctr < 0xFFFF - ROLLING_CODE_TOLERANCE){
		return last_rolling_ctr < current_rolling_ctr && current_rolling_ctr <= (uint16_t) (last_rolling_ctr + ROLLING_CODE_TOLERANCE);
	}
	else {
		return last_rolling_ctr < current_rolling_ctr || current_rolling_ctr <= (uint16_t) (last_rolling_ctr + ROLLING_CODE_TOLERANCE);
	}
}

bool Cetaguard_receiver::is_time_congruent(uint32_t last_time, uint32_t msg_time, uint32_t last_time_recv, uint32_t recv_time){
	switch (clock_source){
		case CETAGUARD_CLOCK_INTERNAL:
    	case CETAGUARD_CLOCK_LOCAL:
    	case CETAGUARD_CLOCK_REMOTE: {
        //IMPORTANT:
        //Message processing time does not need to be taken into account **assuming it is constant**
        int time_diff_transmitter = msg_time - last_time; //time difference at transmitter
        int time_diff_receiver = recv_time - last_time_recv; //time difference at receiver
        int max_error = round(time_diff_transmitter * (TOLERANCE_PPM / 1e6f)) + 1;
        return time_diff_receiver - max_error <= time_diff_transmitter && time_diff_transmitter <= time_diff_receiver + max_error;
    	}
    	break;
	}
	return true;
}

Cetaguard_status Cetaguard_receiver::interpret_message(uint8_t* msg_in, size_t length){
	return interpret_message(msg_in, length, get_time());
}

Cetaguard_status Cetaguard_receiver::interpret_message(uint8_t* msg_in, size_t length, uint32_t recv_time){
	if (length < 4) {
		return CETAGUARD_MSG_TOO_SHORT;
	}
	Cetaguard_preamble* msg_pre = (Cetaguard_preamble*)msg_in;
	if (msg_pre->magic[0] != CETAGUARD_MAGIC_0 ||
		msg_pre->magic[1] != CETAGUARD_MAGIC_1 ||
		msg_pre->magic[2] != CETAGUARD_MAGIC_2) {
		return CETAGUARD_MSG_BAD_PREAMBLE;
	}
	if (msg_pre->version != CETAGUARD_VERSION) {
		return CETAGUARD_MSG_VERSION_UNSUPPORTED;
	}
	switch (msg_pre->msg_type){
		case CETAGUARD_TYPE_BUTTON: {
			Cetaguard_button_message* msg = (Cetaguard_button_message*)msg_in;
			if (length < sizeof(*msg)){
				return CETAGUARD_MSG_TOO_SHORT;
			}
			int idx = find_transmitter(msg->transmitter_id);
			if (idx < 0 || idx >= transmitters.size()){
				return CETAGUARD_MSG_TXR_NOT_PAIRED;
			}
			Cetaguard_paired_transmitter* tp = &(transmitters[idx]);
			debug_data[0] = ESP.getCycleCount();
			//decrypt - from last block to first
			uint8_t* decr = (uint8_t*)&(msg->init_value);
			decr += 2*ENCR_BLOCK_SIZE;

			decrypt(decr, decr, tp->transmission_key);
			for (int i = 0; i < ENCR_BLOCK_SIZE; i++){
				decr[i] ^= decr[i - ENCR_BLOCK_SIZE];
			}
			decr -= ENCR_BLOCK_SIZE;
			
			decrypt(decr, decr, tp->transmission_key);
			for (int i = 0; i < ENCR_BLOCK_SIZE; i++){
				decr[i] ^= decr[i - ENCR_BLOCK_SIZE];
			}
			decr -= ENCR_BLOCK_SIZE;
			decrypt(decr, decr, tp->transmission_key);
			debug_data[1] = ESP.getCycleCount();
			//calculate hmac
			uint8_t hmac_in[sizeof(tp->transmission_key) + sizeof(msg->init_value) + sizeof(msg->transmitter_id) + ENCR_BLOCK_SIZE];
			uint8_t* hmac_in_ptr = hmac_in;
			memcpy(hmac_in_ptr, tp->transmission_key, sizeof(tp->transmission_key));
			hmac_in_ptr += sizeof(tp->transmission_key);
			memcpy(hmac_in_ptr, &msg->init_value, sizeof(msg->init_value));
			hmac_in_ptr += sizeof(msg->init_value);
			memcpy(hmac_in_ptr, &msg->transmitter_id, sizeof(msg->transmitter_id));
			hmac_in_ptr += sizeof(msg->transmitter_id);
			memcpy(hmac_in_ptr, &(msg->time_value), ENCR_BLOCK_SIZE);
			uint8_t hmac_full[HASH_SIZE];
			hash(hmac_in, sizeof(hmac_in), hmac_full);
			uint8_t contents_test = msg->padding_1; //both padding bytes should be 0
			contents_test |= msg->padding_2;
			contents_test |= memcmp(msg->hmac, hmac_full, OUT_HASH_SIZE); //returned value should be 0
			//The reason I don't do if statements above is to avoid creating a possible timing side-channel
			if (contents_test != 0){
				return CETAGUARD_MSG_BAD_MAC; 
			}

			if (!tp->data_initialized || (tp->data_initialized && has_been_reset(tp->reset_ctr, msg->reset_ctr))){
				//If data is not initialized (it's the first message after pairing)
				//the receiver will accept the values received as initial and not react
				tp->last_time = msg->time_value;
				tp->last_time_recv = recv_time;
				tp->reset_ctr = msg->reset_ctr;
				tp->rolling_ctr = msg->rolling_ctr;
				tp->data_initialized = 1;
			}
			else { //tp->data_initialized && !has_been_reset(tp->reset_ctr, msg->reset_ctr)
				int ctr_invalid = 0;
				ctr_invalid |= !is_time_congruent(tp->last_time, msg->time_value, tp->last_time_recv, recv_time);
				ctr_invalid |= !(tp->reset_ctr == msg->reset_ctr);
				ctr_invalid |= !is_counter_congruent(tp->rolling_ctr, msg->rolling_ctr);
				if (ctr_invalid){
					return CETAGUARD_MSG_BAD_COUNTER;
				}
				tp->last_time = msg->time_value;
				tp->last_time_recv = recv_time;
				tp->reset_ctr = msg->reset_ctr;
				tp->rolling_ctr = msg->rolling_ctr;

				Cetaguard_msg_contents contents;
				contents.buttons = msg->btn_press_value;
				contents.pattern = msg->pattern;
				contents.battery = msg->battery;
				interpreter->button_msg(contents, idx);
			}
		}
		break;
		case CETAGUARD_TYPE_PAIRING_NS: {
			Cetaguard_pairing_message* msg = (Cetaguard_pairing_message*)msg_in;
			if (length < sizeof(*msg)){
				return CETAGUARD_MSG_TOO_SHORT;
			}
			if (!get_pairing_mode()){
				return CETAGUARD_NOT_PAIRING;
			}
			
			int idx = find_transmitter(msg->transmitter_id);
			if (idx >= 0 && idx < transmitters.size()){
				return CETAGUARD_MSG_TXR_ALREADY_PAIRED;
			}
			
			uint8_t hmac_in[sizeof(msg->transmitter_id) + 2 * PUB_KEY_SIZE];
			memcpy(hmac_in + 0, &msg->transmitter_id, sizeof(msg->transmitter_id));
			memcpy(hmac_in + sizeof(msg->transmitter_id), msg->pub_key, PUB_KEY_SIZE);
			memcpy(hmac_in + sizeof(msg->transmitter_id) + PUB_KEY_SIZE, static_pub_key, PUB_KEY_SIZE);
			uint8_t hmac_full[HASH_SIZE];
			hash(hmac_in, sizeof(hmac_in), hmac_full);
			if (memcmp(msg->hmac, hmac_full, OUT_HASH_SIZE) != 0){
				return CETAGUARD_MSG_BAD_MAC;
			}
			Cetaguard_paired_transmitter t;
			t.transmitter_id = msg->transmitter_id;
			t.data_initialized = 0;
			memcpy(t.pub_key, static_pub_key, PUB_KEY_SIZE);
			memcpy(t.priv_key, static_priv_key, PRIV_KEY_SIZE);
			uint8_t secret_tmp[SECRET_SIZE];
			if (!calculate_secret(msg->pub_key, t.priv_key, secret_tmp)){
				return CETAGUARD_SECRET_CALC_FAILED;
			}
			uint8_t secret_hash[HASH_SIZE];
			hash(secret_tmp, SECRET_SIZE, secret_hash);
			memcpy(t.transmission_key, secret_hash, ENCR_KEY_SIZE);
			transmitters.push_back(t);
			interpreter->pairing_without_secret_msg(transmitters.size() - 1);
		}
		break;
		case CETAGUARD_TYPE_PAIRING_S: {
			Cetaguard_pairing_message* msg = (Cetaguard_pairing_message*)msg_in;
			if (length < sizeof(*msg)){
				return CETAGUARD_MSG_TOO_SHORT;
			}
			int idx = find_pending(msg->transmitter_id);
			if (idx < 0 || idx >= pending.size()){
				return CETAGUARD_MSG_TXR_NOT_PENDING;
			}
			//TODO: handle case when transmitter with the same transmitter_id is already paired
			Cetaguard_pending_transmitter* pp = &pending[idx];
			uint8_t hmac_in[P_SECRET_SIZE + 2 * PUB_KEY_SIZE];
			memcpy(hmac_in + 0, 			  			 pp->pairing_secret, P_SECRET_SIZE);
			memcpy(hmac_in + P_SECRET_SIZE,				 msg->pub_key, PUB_KEY_SIZE);
			memcpy(hmac_in + P_SECRET_SIZE + PUB_KEY_SIZE, pp->pub_key, PUB_KEY_SIZE);
			uint8_t hmac_full[HASH_SIZE];
			hash(hmac_in, sizeof(hmac_in), hmac_full);
			if (memcmp(msg->hmac, hmac_full, OUT_HASH_SIZE) != 0){
				return CETAGUARD_MSG_BAD_MAC;
			}
			Cetaguard_paired_transmitter t;
			t.transmitter_id = pp->transmitter_id;
			t.data_initialized = 0;
			memcpy(t.pub_key, pp->pub_key, PUB_KEY_SIZE);
			memcpy(t.priv_key, pp->priv_key, PRIV_KEY_SIZE);
			uint8_t secret_tmp[SECRET_SIZE];
			if (!calculate_secret(msg->pub_key, t.priv_key, secret_tmp)){
				return CETAGUARD_SECRET_CALC_FAILED;
			}
			uint8_t secret_hash[HASH_SIZE];
			hash(secret_tmp, SECRET_SIZE, secret_hash);
			memcpy(t.transmission_key, secret_hash, ENCR_KEY_SIZE);
			pending.erase(pending.begin() + idx);
			transmitters.push_back(t);
			interpreter->pairing_with_secret_msg(transmitters.size() - 1);
		}
		break;
		default: return CETAGUARD_MSG_UNKNOWN_TYPE;
	}
	return CETAGUARD_OK;
}

Cetaguard_status Cetaguard_receiver::remove_transmitter(Cetaguard_index idx){
	if (idx >= 0 && idx < transmitters.size()){
		transmitters.erase(transmitters.begin() + idx);
		return CETAGUARD_OK;
	}
	return CETAGUARD_INDEX_OOBOUNDS;
}

Cetaguard_status Cetaguard_receiver::remove_pending(Cetaguard_index idx){
	if (idx >= 0 && idx < pending.size()){
		pending.erase(pending.begin() + idx);
		return CETAGUARD_OK;
	}
	return CETAGUARD_INDEX_OOBOUNDS;
}

Cetaguard_index Cetaguard_receiver::find_transmitter(uint32_t transmitter_id){
	for (int i = 0; i < transmitters.size(); i++){
		if (transmitters[i].transmitter_id == transmitter_id)
			return i;
	}
	return -1;
}

Cetaguard_index Cetaguard_receiver::find_pending(uint32_t secret_hash){
	for (int i = 0; i < pending.size(); i++){
		if (pending[i].secret_hash == secret_hash)
			return i;
	}
	return -1;
}

void Cetaguard_receiver::set_pairing_mode(bool on){
	in_pairing_mode = on;
}

bool Cetaguard_receiver::get_pairing_mode(){
	return in_pairing_mode;
}

void Cetaguard_receiver::time_was_synchronized(){
	time_at_last_sync = get_time();
}

/*===== TRANSMITTER CODE =======*/
Cetaguard_transmitter::Cetaguard_transmitter(){
}

Cetaguard_status Cetaguard_transmitter::add_receiver(const uint8_t* recv_pub_key){
	Cetaguard_pending_receiver p;
	p.pairing_secret_used = 0;
	memcpy(p.receiver_pub_key, recv_pub_key, PUB_KEY_SIZE);
	if (!generate_keys(p.pub_key, p.priv_key)){
		return CETAGUARD_KEY_GEN_FAILED;
	}
	//TODO: the value generated below can be duplicate!!!
	if (!generate_random((uint8_t*)&p.transmitter_id, sizeof(p.transmitter_id))){
		return CETAGUARD_RANDOM_FAILED;
	}
	pending.push_back(p);
	return CETAGUARD_OK;
}

Cetaguard_status Cetaguard_transmitter::add_receiver(const uint8_t* recv_pub_key, const uint8_t* pairing_secret){
	Cetaguard_pending_receiver p;
	p.pairing_secret_used = 1;
	memcpy(p.receiver_pub_key, recv_pub_key, PUB_KEY_SIZE);
	memcpy(p.pairing_secret, pairing_secret, P_SECRET_SIZE);
	uint8_t secret_hash[HASH_SIZE];
	hash(p.pairing_secret, P_SECRET_SIZE, secret_hash);
	p.secret_hash = *(uint32_t*)secret_hash;
	p.transmitter_id = *(uint32_t*)(secret_hash + HASH_SIZE - sizeof(uint32_t));
	if (!generate_keys(p.pub_key, p.priv_key)){
		return CETAGUARD_KEY_GEN_FAILED;
	}
	pending.push_back(p);
	return CETAGUARD_OK;
}

Cetaguard_status Cetaguard_transmitter::prepare_pairing_msg(Cetaguard_index idx, uint8_t* msg_out, size_t* size_out){
	if (idx < 0 || idx >= pending.size()){
		return CETAGUARD_INDEX_OOBOUNDS;
	}
	Cetaguard_pending_receiver* pp = &pending[idx];
	Cetaguard_pairing_message* msg = (Cetaguard_pairing_message* )msg_out;
	*size_out = sizeof(Cetaguard_pairing_message);
	msg->pre.magic[0] = CETAGUARD_MAGIC_0;
	msg->pre.magic[1] = CETAGUARD_MAGIC_1;
	msg->pre.magic[2] = CETAGUARD_MAGIC_2;
	msg->pre.version  = CETAGUARD_VERSION;
	msg->pre.msg_type = pp->pairing_secret_used ? CETAGUARD_TYPE_PAIRING_S : CETAGUARD_TYPE_PAIRING_NS;
	msg->transmitter_id = pp->pairing_secret_used ? pp->secret_hash : pp->transmitter_id;
	memcpy(msg->pub_key, pp->pub_key, PUB_KEY_SIZE);
	if (pp->pairing_secret_used){
		uint8_t hmac_in[P_SECRET_SIZE + 2 * PUB_KEY_SIZE];
		memcpy(hmac_in + 0, 			  			 pp->pairing_secret, P_SECRET_SIZE);
		memcpy(hmac_in + P_SECRET_SIZE,				 pp->pub_key, PUB_KEY_SIZE);
		memcpy(hmac_in + P_SECRET_SIZE + PUB_KEY_SIZE, pp->receiver_pub_key, PUB_KEY_SIZE);
		uint8_t hmac_full[HASH_SIZE];
		hash(hmac_in, sizeof(hmac_in), hmac_full);
		memcpy(msg->hmac, hmac_full, OUT_HASH_SIZE);
	}
	else {
		uint8_t hmac_in[sizeof(msg->transmitter_id) + 2 * PUB_KEY_SIZE];
		memcpy(hmac_in + 0, &pp->transmitter_id, sizeof(msg->transmitter_id));
		memcpy(hmac_in + sizeof(msg->transmitter_id), pp->pub_key, PUB_KEY_SIZE);
		memcpy(hmac_in + sizeof(msg->transmitter_id) + PUB_KEY_SIZE, pp->receiver_pub_key, PUB_KEY_SIZE);
		uint8_t hmac_full[HASH_SIZE];
		hash(hmac_in, sizeof(hmac_in), hmac_full);
		memcpy(msg->hmac, hmac_full, OUT_HASH_SIZE);
	}
	return CETAGUARD_OK;
}

Cetaguard_status Cetaguard_transmitter::finish_pairing(Cetaguard_index idx){
	if (idx < 0 || idx >= pending.size()){
		return CETAGUARD_INDEX_OOBOUNDS;
	}
	Cetaguard_paired_receiver r;
	Cetaguard_pending_receiver* pp = &pending[idx];
	r.transmitter_id = pp->transmitter_id;
	r.reset_ctr = 0;
	r.rolling_ctr = 0;
	memcpy(r.pub_key, pp->pub_key, PUB_KEY_SIZE);
	memcpy(r.priv_key, pp->priv_key, PRIV_KEY_SIZE);
	memcpy(r.receiver_pub_key, pp->receiver_pub_key, PUB_KEY_SIZE);
	uint8_t secret_tmp[SECRET_SIZE];
	if (!calculate_secret(r.receiver_pub_key, r.priv_key, secret_tmp)){
		return CETAGUARD_SECRET_CALC_FAILED;
	}
	uint8_t secret_hash[HASH_SIZE];
	hash(secret_tmp, SECRET_SIZE, secret_hash);
	memcpy(r.transmission_key, secret_hash, ENCR_KEY_SIZE);
	pending.erase(pending.begin() + idx);
	receivers.push_back(r);
	return CETAGUARD_OK;
}

Cetaguard_status Cetaguard_transmitter::prepare_button_msg(Cetaguard_index idx, const Cetaguard_msg_contents* contents, uint8_t* msg_out, size_t* size_out){
	if (idx < 0 || idx >= receivers.size()){
		return CETAGUARD_INDEX_OOBOUNDS;
	}
	Cetaguard_paired_receiver* rp = &receivers[idx];
	Cetaguard_button_message* msg = (Cetaguard_button_message*) msg_out;
	*size_out = sizeof(Cetaguard_button_message);
	debug_data[0] = ESP.getCycleCount();
	//prepare contents
	msg->pre.magic[0] = CETAGUARD_MAGIC_0;
	msg->pre.magic[1] = CETAGUARD_MAGIC_1;
	msg->pre.magic[2] = CETAGUARD_MAGIC_2;
	msg->pre.version  = CETAGUARD_VERSION;
	msg->pre.msg_type = CETAGUARD_TYPE_BUTTON;
	msg->transmitter_id = rp->transmitter_id;
	if (!generate_random((uint8_t*)&msg->init_value, sizeof(msg->init_value))){
		return CETAGUARD_RANDOM_FAILED;
	}
	msg->time_value = get_time();
	msg->btn_press_value = contents->buttons;
	msg->reset_ctr = rp->reset_ctr;
	msg->rolling_ctr = (rp->rolling_ctr)++;
	msg->pattern = contents->pattern;
	msg->battery = contents->battery;
	msg->padding_1 = 0;
	msg->padding_2 = 0;
	debug_data[1] = ESP.getCycleCount();
	//calculate hmac
	uint8_t hmac_in[sizeof(rp->transmission_key) + sizeof(msg->init_value) + sizeof(msg->transmitter_id) + ENCR_BLOCK_SIZE];
	uint8_t* hmac_in_ptr = hmac_in;
	memcpy(hmac_in_ptr, rp->transmission_key, sizeof(rp->transmission_key));
	hmac_in_ptr += sizeof(rp->transmission_key);
	memcpy(hmac_in_ptr, &msg->init_value, sizeof(msg->init_value));
	hmac_in_ptr += sizeof(msg->init_value);
	memcpy(hmac_in_ptr, &msg->transmitter_id, sizeof(msg->transmitter_id));
	hmac_in_ptr += sizeof(msg->transmitter_id);
	memcpy(hmac_in_ptr, &msg->time_value, ENCR_BLOCK_SIZE); //message block 2 starts at time_value
	uint8_t hmac_full[HASH_SIZE];
	hash(hmac_in, sizeof(hmac_in), hmac_full);
	memcpy(msg->hmac, hmac_full, OUT_HASH_SIZE);
	debug_data[2] = ESP.getCycleCount();
	//encrypt
	uint8_t* encr_data = (uint8_t*)&(msg->init_value);
	encrypt(encr_data, encr_data, rp->transmission_key);
	for (int i = 0; i < ENCR_BLOCK_SIZE; i++){
		encr_data[i + ENCR_BLOCK_SIZE] ^= encr_data[i];
	}
	encr_data += ENCR_BLOCK_SIZE;
	encrypt(encr_data, encr_data, rp->transmission_key);
	for (int i = 0; i < ENCR_BLOCK_SIZE; i++){
		encr_data[i + ENCR_BLOCK_SIZE] ^= encr_data[i];
	}
	encr_data += ENCR_BLOCK_SIZE;
	encrypt(encr_data, encr_data, rp->transmission_key);
	debug_data[3] = ESP.getCycleCount();
	return CETAGUARD_OK;
}

Cetaguard_status Cetaguard_transmitter::remove_receiver(Cetaguard_index idx){
	if (idx >= 0 && idx < receivers.size()){
		receivers.erase(receivers.begin() + idx);
		return CETAGUARD_OK;
	}
	return CETAGUARD_INDEX_OOBOUNDS;
}

Cetaguard_status Cetaguard_transmitter::remove_pending(Cetaguard_index idx){
	if (idx >= 0 && idx < pending.size()){
		pending.erase(pending.begin() + idx);
		return CETAGUARD_OK;
	}
	return CETAGUARD_INDEX_OOBOUNDS;
}

void Cetaguard_transmitter::increment_reset_counters(){
	for (int i = 0; i < receivers.size(); i++){
		receivers[i].reset_ctr++;
		receivers[i].rolling_ctr = 0;
	}
}
