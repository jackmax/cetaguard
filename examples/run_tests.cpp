#include <iostream>
#include <iomanip>
#include <stdlib.h>
#include "cetaguard.h"
#include <unistd.h>
#include <vector>

using namespace std;


class My_interpreter: public Cetaguard_message_interpreter{
	void button_msg(Cetaguard_msg_contents contents, Cetaguard_index idx){
		cout << "Button message (" <<
		(int)contents.battery << ", " <<
		contents.buttons << ", " << 
		(int)contents.pattern << ")" << endl;
	}
    void pairing_without_secret_msg(Cetaguard_index idx){
    	cout << "Pairing ns message" << endl;
    }
    void pairing_with_secret_msg(Cetaguard_index idx){
    	cout << "Pairing s message" << endl;
	}
};



void print_hex(uint8_t* data, size_t size){
	cout << right <<  hex;
	for (size_t i = 0; i < size; i++){
		cout << setw(2) << setfill('0') << (int)data[i] << " ";
		if (i % 16 == 15){
			cout << endl;
		}
	}
	cout << right <<  dec;
}

class Storage_test: public Cetaguard_exporter_importer
{
public:
	size_t read_i;
	vector<uint8_t> array;

    Storage_test(){
    	read_i = 0;
    }

    void rewind(){
    	read_i = 0;
    }

    virtual bool read_byte(uint8_t* byte_out){
        if (read_i >= array.size()){
        	return false;
        }
        *byte_out = array[read_i++];
        return true;
    }
    virtual bool write_byte(uint8_t byte){
    	array.push_back(byte);
    	return true;
    }
};

int main(){
	srand(1234);
	Cetaguard_transmitter tx;
	My_interpreter interpreter;
	//Random number generator has to be initialised before the transmitter object can be generated
	Cetaguard_receiver rx = Cetaguard_receiver(&interpreter);
	rx.set_clock_source(CETAGUARD_CLOCK_INTERNAL);
	rx.set_pairing_mode(true);

	cout << "TX add pending recv " << tx.add_receiver(rx.get_public_key_static()) << endl;
	uint8_t msg1[512], msg2[512];
	size_t size1, size2;
	cout << "TX pairing message, result " << tx.prepare_pairing_msg(0, msg1, &size1) << endl;
	print_hex(msg1, size1);
	cout << endl;
	rx.set_pairing_mode(true);
	cout << "RX message interpreted, result " << rx.interpret_message(msg1, size1) << endl;
	rx.set_pairing_mode(false);
	cout << "TX finish pairing recv " << tx.finish_pairing(0) << endl;
	Cetaguard_msg_contents c_in;
	c_in.buttons = 0xDEADCAFE;
	c_in.battery = 0xBA;
	c_in.pattern = 0xFC;
	cout << "TX prepare msg " << tx.prepare_button_msg(0, &c_in, msg1, &size1) << endl;
	print_hex(msg1, size1);
	cout << endl;
	cout << "RX message interpreted, result " << rx.interpret_message(msg1, size1) << endl;

	cout << "TX prepare msg " << tx.prepare_button_msg(0, &c_in, msg1, &size1) << endl;
	print_hex(msg1, size1);
	cout << endl;
	cout << "RX message interpreted, result " << rx.interpret_message(msg1, size1) << endl;
	
	uint32_t id2;
	cout << "RX add pending, result " << rx.add_pending_transmitter(&id2) << endl;
	Cetaguard_index idx2 = rx.find_pending(id2);
	const uint8_t* secret = rx.get_pairing_secret(idx2);
	const uint8_t* pubkey = rx.get_public_key_pending(idx2);
	tx.add_receiver(pubkey, secret);
	
	cout << "TX pairing message, result " << tx.prepare_pairing_msg(0, msg2, &size2) << endl;
	print_hex(msg2, size2);
	cout << endl;
	cout << "RX message interpreted, result " << rx.interpret_message(msg2, size2) << endl;
	cout << "TX finish pairing recv " << tx.finish_pairing(0) << endl;
	Cetaguard_msg_contents c2_in;
	c2_in.buttons = 0xDEEDCAFE;
	c2_in.battery = 0xAB;
	c2_in.pattern = 0xCF;
	cout << "TX prepare msg " << tx.prepare_button_msg(1, &c2_in, msg2, &size2) << endl;
	print_hex(msg2, size2);
	cout << endl;
	cout << "RX message interpreted, result " << rx.interpret_message(msg2, size2) << endl;
	
	cout << "TX prepare msg " << tx.prepare_button_msg(1, &c2_in, msg2, &size2) << endl;
	print_hex(msg2, size2);
	cout << endl;
	cout << "RX message interpreted, result " << rx.interpret_message(msg2, size2) << endl;
	
	cout << "Transmitter reset test" << endl;
	tx.increment_reset_counters();
	
	cout << "TX prepare msg " << tx.prepare_button_msg(1, &c2_in, msg2, &size2) << endl;
	print_hex(msg2, size2);
	cout << endl;
	cout << "RX message interpreted, result " << rx.interpret_message(msg2, size2) << endl;
	
	cout << "TX prepare msg " << tx.prepare_button_msg(1, &c2_in, msg2, &size2) << endl;
	print_hex(msg2, size2);
	cout << endl;
	cout << "RX message interpreted, result " << rx.interpret_message(msg2, size2) << endl;
	
	cout << "Rolling counter test" << endl;
	tx.receivers[1].rolling_ctr += ROLLING_CODE_TOLERANCE - 1; //simulate that many messages sent and lost
	
	cout << "TX prepare msg " << tx.prepare_button_msg(1, &c2_in, msg2, &size2) << endl;
	print_hex(msg2, size2);
	cout << endl;
	cout << "RX message interpreted, result " << rx.interpret_message(msg2, size2) << endl;
	
	cout << "TX prepare msg " << tx.prepare_button_msg(1, &c2_in, msg2, &size2) << endl;
	print_hex(msg2, size2);
	cout << endl;
	cout << "RX message interpreted, result " << rx.interpret_message(msg2, size2) << endl;
	
	tx.receivers[1].rolling_ctr = 10 + 1;
	rx.transmitters[1].rolling_ctr = 10 - (ROLLING_CODE_TOLERANCE - 1);
	
	cout << "TX prepare msg " << tx.prepare_button_msg(1, &c2_in, msg2, &size2) << endl;
	print_hex(msg2, size2);
	cout << endl;
	cout << "RX message interpreted, result " << rx.interpret_message(msg2, size2) << endl;
	
	cout << "Timer test, waiting 5 seconds" << endl;
	//sleep(5);
	
	cout << "TX prepare msg " << tx.prepare_button_msg(1, &c2_in, msg2, &size2) << endl;
	print_hex(msg2, size2);
	cout << endl;
	cout << "RX message interpreted, result " << rx.interpret_message(msg2, size2) << endl;
	
	Storage_test storage_tx;
	Storage_test storage_rx;
	bool result = tx.export_state(&storage_tx);
	result = result && rx.export_state(&storage_rx);
	if (!result){
		return 1;
	}
	cout << "TX, RX stored" << endl;

	Cetaguard_transmitter tx2(&storage_tx);
	Cetaguard_receiver rx2(&interpreter, &storage_rx);

	cout << "TX, RX restored" << endl;

	cout << "TX2 prepare msg " << tx2.prepare_button_msg(1, &c2_in, msg2, &size2) << endl;
	print_hex(msg2, size2);
	cout << endl;
	cout << "RX2 message interpreted, result " << rx2.interpret_message(msg2, size2) << endl;

	return 0;
}
