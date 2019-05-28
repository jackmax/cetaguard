#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServerSecure.h>
#include <ESP8266mDNS.h>
#include <SPI.h>
#include <FS.h>
#include <coredecls.h>
#include <Ticker.h>
#include <DNSServer.h>
#include <IRremoteESP8266.h>
#include <LT8920.h>
#include <deque>
#include <map>

#include "EndUserSetup.h"
#include "cetaguard.h"

extern "C" {
  #include "crc32.h"
}

volatile uint32_t debug_data[64];
volatile uint32_t debug_data_ctr;

std::map<Cetaguard_index, uint8_t> tx_battery_status;

// The certificate is stored in PMEM
static const uint8_t x509[] PROGMEM = {
  0x30, 0x82, 0x01, 0x3d, 0x30, 0x81, 0xe8, 0x02, 0x09, 0x00, 0xfe, 0x56,
  0x46, 0xf2, 0x78, 0xc6, 0x51, 0x17, 0x30, 0x0d, 0x06, 0x09, 0x2a, 0x86,
  0x48, 0x86, 0xf7, 0x0d, 0x01, 0x01, 0x0b, 0x05, 0x00, 0x30, 0x26, 0x31,
  0x10, 0x30, 0x0e, 0x06, 0x03, 0x55, 0x04, 0x0a, 0x0c, 0x07, 0x45, 0x53,
  0x50, 0x38, 0x32, 0x36, 0x36, 0x31, 0x12, 0x30, 0x10, 0x06, 0x03, 0x55,
  0x04, 0x03, 0x0c, 0x09, 0x31, 0x32, 0x37, 0x2e, 0x30, 0x2e, 0x30, 0x2e,
  0x31, 0x30, 0x1e, 0x17, 0x0d, 0x31, 0x37, 0x30, 0x33, 0x31, 0x38, 0x31,
  0x34, 0x34, 0x39, 0x31, 0x38, 0x5a, 0x17, 0x0d, 0x33, 0x30, 0x31, 0x31,
  0x32, 0x35, 0x31, 0x34, 0x34, 0x39, 0x31, 0x38, 0x5a, 0x30, 0x26, 0x31,
  0x10, 0x30, 0x0e, 0x06, 0x03, 0x55, 0x04, 0x0a, 0x0c, 0x07, 0x45, 0x53,
  0x50, 0x38, 0x32, 0x36, 0x36, 0x31, 0x12, 0x30, 0x10, 0x06, 0x03, 0x55,
  0x04, 0x03, 0x0c, 0x09, 0x31, 0x32, 0x37, 0x2e, 0x30, 0x2e, 0x30, 0x2e,
  0x31, 0x30, 0x5c, 0x30, 0x0d, 0x06, 0x09, 0x2a, 0x86, 0x48, 0x86, 0xf7,
  0x0d, 0x01, 0x01, 0x01, 0x05, 0x00, 0x03, 0x4b, 0x00, 0x30, 0x48, 0x02,
  0x41, 0x00, 0xc6, 0x72, 0x6c, 0x12, 0xe1, 0x20, 0x4d, 0x10, 0x0c, 0xf7,
  0x3a, 0x2a, 0x5a, 0x49, 0xe2, 0x2d, 0xc9, 0x7a, 0x63, 0x1d, 0xef, 0xc6,
  0xbb, 0xa3, 0xd6, 0x6f, 0x59, 0xcb, 0xd5, 0xf6, 0xbe, 0x34, 0x83, 0x33,
  0x50, 0x80, 0xec, 0x49, 0x63, 0xbf, 0xee, 0x59, 0x94, 0x67, 0x8b, 0x8d,
  0x81, 0x85, 0x23, 0x24, 0x06, 0x52, 0x76, 0x55, 0x9d, 0x18, 0x09, 0xb3,
  0x3c, 0x10, 0x40, 0x05, 0x01, 0xf3, 0x02, 0x03, 0x01, 0x00, 0x01, 0x30,
  0x0d, 0x06, 0x09, 0x2a, 0x86, 0x48, 0x86, 0xf7, 0x0d, 0x01, 0x01, 0x0b,
  0x05, 0x00, 0x03, 0x41, 0x00, 0x69, 0xdc, 0x6c, 0x9b, 0xa7, 0x62, 0x57,
  0x7e, 0x03, 0x01, 0x45, 0xad, 0x9a, 0x83, 0x90, 0x3a, 0xe7, 0xdf, 0xe8,
  0x8f, 0x46, 0x00, 0xd3, 0x5f, 0x2b, 0x0a, 0xde, 0x92, 0x1b, 0xc5, 0x04,
  0xc5, 0xc0, 0x76, 0xf4, 0xf6, 0x08, 0x36, 0x97, 0x27, 0x82, 0xf1, 0x60,
  0x76, 0xc2, 0xcd, 0x67, 0x6c, 0x4b, 0x6c, 0xca, 0xfd, 0x97, 0xfd, 0x33,
  0x9e, 0x12, 0x67, 0x6b, 0x98, 0x7e, 0xd5, 0x80, 0x8f
};

// And so is the key.  These could also be in DRAM
static const uint8_t rsakey[] PROGMEM = {
  0x30, 0x82, 0x01, 0x3a, 0x02, 0x01, 0x00, 0x02, 0x41, 0x00, 0xc6, 0x72,
  0x6c, 0x12, 0xe1, 0x20, 0x4d, 0x10, 0x0c, 0xf7, 0x3a, 0x2a, 0x5a, 0x49,
  0xe2, 0x2d, 0xc9, 0x7a, 0x63, 0x1d, 0xef, 0xc6, 0xbb, 0xa3, 0xd6, 0x6f,
  0x59, 0xcb, 0xd5, 0xf6, 0xbe, 0x34, 0x83, 0x33, 0x50, 0x80, 0xec, 0x49,
  0x63, 0xbf, 0xee, 0x59, 0x94, 0x67, 0x8b, 0x8d, 0x81, 0x85, 0x23, 0x24,
  0x06, 0x52, 0x76, 0x55, 0x9d, 0x18, 0x09, 0xb3, 0x3c, 0x10, 0x40, 0x05,
  0x01, 0xf3, 0x02, 0x03, 0x01, 0x00, 0x01, 0x02, 0x40, 0x35, 0x0b, 0x74,
  0xd3, 0xff, 0x15, 0x51, 0x44, 0x0f, 0x13, 0x2e, 0x9b, 0x0f, 0x93, 0x5c,
  0x3f, 0xfc, 0xf1, 0x17, 0xf9, 0x72, 0x94, 0x5e, 0xa7, 0xc6, 0xb3, 0xf0,
  0xfe, 0xc9, 0x6c, 0xb1, 0x1e, 0x83, 0xb3, 0xc6, 0x45, 0x3a, 0x25, 0x60,
  0x7c, 0x3d, 0x92, 0x7d, 0x53, 0xec, 0x49, 0x8d, 0xb5, 0x45, 0x10, 0x99,
  0x9b, 0xc6, 0x22, 0x3a, 0x68, 0xc7, 0x13, 0x4e, 0xb6, 0x04, 0x61, 0x21,
  0x01, 0x02, 0x21, 0x00, 0xea, 0x8c, 0x21, 0xd4, 0x7f, 0x3f, 0xb6, 0x91,
  0xfa, 0xf8, 0xb9, 0x2d, 0xcb, 0x36, 0x36, 0x02, 0x5f, 0xf0, 0x0c, 0x6e,
  0x87, 0xaa, 0x5c, 0x14, 0xf6, 0x56, 0x8e, 0x12, 0x92, 0x25, 0xde, 0xb3,
  0x02, 0x21, 0x00, 0xd8, 0x99, 0x01, 0xf1, 0x04, 0x0b, 0x98, 0xa3, 0x71,
  0x56, 0x1d, 0xea, 0x6f, 0x45, 0xd1, 0x36, 0x70, 0x76, 0x8b, 0xab, 0x69,
  0x30, 0x58, 0x9c, 0xe0, 0x45, 0x97, 0xe7, 0xb6, 0xb5, 0xef, 0xc1, 0x02,
  0x21, 0x00, 0xa2, 0x01, 0x06, 0xc0, 0xf2, 0xdf, 0xbc, 0x28, 0x1a, 0xb4,
  0xbf, 0x9b, 0x5c, 0xd8, 0x65, 0xf7, 0xbf, 0xf2, 0x5b, 0x73, 0xe0, 0xeb,
  0x0f, 0xcd, 0x3e, 0xd5, 0x4c, 0x2e, 0x91, 0x99, 0xec, 0xb7, 0x02, 0x20,
  0x4b, 0x9d, 0x46, 0xd7, 0x3c, 0x01, 0x4c, 0x5d, 0x2a, 0xb0, 0xd4, 0xaa,
  0xc6, 0x03, 0xca, 0xa0, 0xc5, 0xac, 0x2c, 0xe0, 0x3f, 0x4d, 0x98, 0x71,
  0xd3, 0xbd, 0x97, 0xe5, 0x55, 0x9c, 0xb8, 0x41, 0x02, 0x20, 0x02, 0x42,
  0x9f, 0xd1, 0x06, 0x35, 0x3b, 0x42, 0xf5, 0x64, 0xaf, 0x6d, 0xbf, 0xcd,
  0x2c, 0x3a, 0xcd, 0x0a, 0x9a, 0x4d, 0x7c, 0xad, 0x29, 0xd6, 0x36, 0x57,
  0xd5, 0xdf, 0x34, 0xeb, 0x26, 0x03
};

static const int MAX_LOG_SIZE = 32;
std::deque<String> message_log;

void add_to_log(String s){
  if (message_log.size() >= MAX_LOG_SIZE){
    message_log.pop_back();
  }
  message_log.push_front(s);
}

void clear_log(){
  while (!message_log.empty()){
    message_log.pop_back();
  }
}

ESP8266WebServerSecure server(443);
//ESP8266WebServer server(80);
const int led = D4;


void handleRoot() {
  digitalWrite(led, 1);
  server.send(200, "text/plain", F("Hello from esp8266 over HTTPS!"));
  digitalWrite(led, 0);
}

void handleNotFound() {
  digitalWrite(led, 1);
  String message = F("File Not Found\n\n");
  message += F("URI: ");
  message += server.uri();
  message += F("\r\nMethod: ");
  message += (server.method() == HTTP_GET) ? F("GET") : F("POST");
  message += F("\r\nArguments: ");
  message += server.args();
  message += '\n';
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
  digitalWrite(led, 0);
}

#define USER_SETUP D0
LT8920 lt(D8, D1, D0);

volatile uint8_t expander = 0;



class My_interpreter: public Cetaguard_message_interpreter{
  void button_msg(Cetaguard_msg_contents contents, Cetaguard_index idx){
    Serial.print(F("Button message ("));
    Serial.print((int)contents.battery, HEX);
    Serial.print(", ");
    Serial.print(contents.buttons, HEX);
    Serial.println(")");
    char buf[16];
    snprintf(buf, 16, "%08x", contents.buttons);
    add_to_log("Time " + String(time(NULL)) + " action " + buf);
    tx_battery_status.insert(std::pair<Cetaguard_index, uint8_t>(idx,contents.battery));
  }
    void pairing_without_secret_msg(Cetaguard_index idx){
      Serial.println(F("Pairing ns message"));
    }
    void pairing_with_secret_msg(Cetaguard_index idx){
      Serial.println(F("Pairing s message"));
  }
};

My_interpreter interpreter;
Cetaguard_receiver* p_receiver;

void printHex(const uint8_t key[], size_t s, String* in_out){
  char buf[4];
  for (int i = 0; i < s; i++){
    snprintf(buf, 4, "%02X", key[i]);
    (*in_out) += buf;
  }
}
/*
 * {
 *  "spubkey" : "RECEIVER_STATIC_PUBLIC_KEY",
 *  "tx": [
 *    {
 *      "txid":123
 *      "last_time":123
        "last_time_recv":123
        "rx_pub_key":
 *    },
 *    {
 *      ...
 *    }
 *  ]
 *  "ptx": [
 *    {
 *      pairing_secret[SECRET_SIZE];
        pub_key[PUB_KEY_SIZE];
      }
 *  ]
 * }
 */
void handleGetStatus() {
  digitalWrite(led, 1);
  String message(F("{\"spubkey\":\""));
  printHex(p_receiver->static_pub_key, PUB_KEY_SIZE, &message);
  message += F("\",\"tx\":[");
  for (unsigned i = 0; i < p_receiver->transmitters.size(); i++){
    if (i != 0){ message += ','; }
    message += F("{\"txid\":");
    message += String(p_receiver->transmitters[i].transmitter_id);
    message += F(",\"lt\":");
    message += String(p_receiver->transmitters[i].last_time);
    message += F(",\"ltr\":");
    message += String(p_receiver->transmitters[i].last_time_recv);
    auto batt_it = tx_battery_status.find(i);
    if (batt_it != tx_battery_status.end()){
      message += F(",\"bat\":");
      message += String(batt_it->second);
    }
    message += F(",\"rpk\":\"");
    printHex(p_receiver->transmitters[i].pub_key, PUB_KEY_SIZE, &message);
    message += F("\"}");
  }
  message += F("],\"ptx\":[");
  for (unsigned i = 0; i < p_receiver->pending.size(); i++){
    if (i != 0){ message += ','; }
    message += F("{\"ps\":\"");
    printHex(p_receiver->pending[i].pairing_secret, SECRET_SIZE, &message);
    message += F("\",\"rpk\":\"");
    printHex(p_receiver->pending[i].pub_key, PUB_KEY_SIZE, &message);
    message += F("\"}");
  }
  message += F("]}\r\n");
  server.send(200, "application/json", message);
  digitalWrite(led, 0);
}

void handleAddPending(){
  if (server.method() != HTTP_POST){
    server.send(405, "application/json", F("{\"error\":\"This endpoint only accepts POST requests\"}"));
    return;
  }
  uint32_t id = 0xFFFFFFFF;
  Cetaguard_status retcode = p_receiver->add_pending_transmitter(&id);
  Cetaguard_index i = p_receiver->find_pending(id);
  if (retcode != CETAGUARD_OK || i < 0){
    String message(F("{\"errcode\":"));
    message += String(retcode);
    message += '}';
    server.send(500, "application/json", message);
    return;
  }
  String message(F("{\"ps\":\""));
  printHex(p_receiver->pending[i].pairing_secret, SECRET_SIZE, &message);
  message += F("\",\"rpk\":\"");
  printHex(p_receiver->pending[i].pub_key, PUB_KEY_SIZE, &message);
  message += F("\"}");
  server.send(200, "application/json", message);
}

void json_escape_string(String in, String* s){
  int len = in.length();
  for (int j = 0; j < len; j++){
    char c = in[j];
    if (c < 0x20 || c == 0x7F){
      char buf[8];
      snprintf(buf, 8, "\\u%02x", c);
      (*s) += buf;
    }
    else if (c == '\\'){
      (*s) += "\\\\";
    }
    else if (c == '"'){
      (*s) += "\\\"";
    }
    else {
      (*s) += c;
    }
  }
}

void print_log_json(String* s){
  (*s) += '[';
  int log_size = message_log.size();
  for (int i = 0; i < log_size; i++){
    if (i != 0){
      (*s) += ',';
    }
    (*s) += '"';
    json_escape_string(message_log[i], s);
    (*s) += '"';
  }
  (*s) += ']';
}

void handleGetLog(){
  if (server.method() != HTTP_GET){
    server.send(405, "application/json", F("{\"error\":\"This endpoint only accepts GET requests\"}"));
    return;
  }
  String message;
  print_log_json(&message);
  server.send(200, "application/json", message);
}

volatile bool time_set = false;

void time_is_set(void) {
  Serial.println(F("settimeofday() was called"));
  time_set = true;
}

void setup(void) {
  timeval tv = { 0, 0 };
  timezone tz = { 0, 0 };
  settimeofday(&tv, &tz);
  
  pinMode(led, OUTPUT);
  pinMode(D8, OUTPUT);
  pinMode(USER_SETUP, INPUT_PULLDOWN_16);
  digitalWrite(led, 0);
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(); //start wifi here and so some other stuff in the meantime
  Serial.println();
  SPIFFS.begin();
  Serial.println(F("SPIFFS file list:"));
  {
    Dir dir = SPIFFS.openDir("/");
    while (dir.next()) {    
      String fileName = dir.fileName();
      size_t fileSize = dir.fileSize();
      Serial.printf("File: %s, size: %d\n", fileName.c_str(), fileSize);
    }
    Serial.println("");
  }

  if (digitalRead(USER_SETUP)){
    Serial.println(F("Setup pressed"));
    performEndUserSetup();
    return;
  }
  pinMode(USER_SETUP, OUTPUT);
  p_receiver = new Cetaguard_receiver(&interpreter);
  p_receiver->set_pairing_mode(true);
  p_receiver->clock_source = CETAGUARD_CLOCK_INTERNAL;
  
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1)); 
  lt.begin();
  //lt.setCurrentControl(0,0);  
  //lt.setDataRate(LT8920::LT8920_1MBPS); 
  lt.setChannel(0x20);

  //verify chip registers.
  for (int i = 0; i <= 50; i++)
  {
    Serial.printf("%d = %04x", i, lt.readRegister(i));
    Serial.println();
  }
  
  // Wait for connection
  int timeout = 15;
  while (WiFi.status() != WL_CONNECTED && timeout-- > 0) {
    delay(500);
    Serial.print(".");
    yield();
  }
  Serial.println();
  Serial.print(F("IP address: "));
  Serial.println(WiFi.localIP());
  
  settimeofday_cb(time_is_set);
  configTime(0, 0, "0.pool.ntp.org");
  
  if (MDNS.begin("esp8266")) {
    Serial.println(F("MDNS responder started"));
  }

  server.setServerKeyAndCert_P(rsakey, sizeof(rsakey), x509, sizeof(x509));

  server.on("/", handleRoot);
  server.on("/status", handleGetStatus);
  server.on("/add_pending", handleAddPending);
  server.on("/log", handleGetLog);
/*
  server.on("/inline", []() {
    server.send(200, "text/plain", "this works as well");
  });
*/
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println(F("HTTPS server started"));

  lt.startListening();
}

void sendPublicKey(){
  uint8_t buffer[PUB_KEY_SIZE+8];
  buffer[0] = 0x01;
  buffer[1] = 0x02;
  buffer[2] = 0x03;
  buffer[3] = 0x18;
  memcpy(buffer+8, p_receiver->static_pub_key, PUB_KEY_SIZE);
  ((uint32_t*)buffer)[1] = crc32(CRC_INIT, buffer+8, PUB_KEY_SIZE);
  lt.sendPacket(buffer, PUB_KEY_SIZE + 8);
}

void loop(void) {
  server.handleClient();
  MDNS.update();
  while (Serial.available()){
    int c = Serial.read();
    if (c == 'p'){
      Serial.println(F("Sending pubkey"));
      sendPublicKey();
      lt.startListening();
    }
  }
  if (lt.available()) {
    uint8_t buffer[64];
    int length = lt.read(buffer, 64);
    if (length < 1){
      Serial.println(F("Error receiving"));
      lt.whatsUp(Serial);
    }
    else if (buffer[3] == 0x18){
      Serial.println(F("Received pubkey request"));
      sendPublicKey();
    }
    else {
      Serial.print(F("Received at "));
      time_t recvTime = lt._recv_time;
      Serial.println(recvTime);
      for (int i = 0; i < length; i++){
        Serial.print(buffer[i] < 0x10 ? " 0" : " ");
        Serial.print(buffer[i], HEX);
      }
      Serial.println();
      Cetaguard_status r = p_receiver->interpret_message(buffer, length, recvTime);
      Serial.println(debug_data[0]);
      Serial.println(debug_data[1]);
      Serial.println(debug_data[2]);
      Serial.println(debug_data[3]);
      Serial.print(F("Return code: "));
      Serial.println(r);
      if (r != CETAGUARD_OK){
        String message(F("Error interpreting message at "));
        message += String(recvTime);
        message += F(" code: ");
        message += String(r);
        if (length == 4){
          message += " (";
          message += String(*(uint32_t*)buffer);
          message += ")";
        }
        add_to_log(message);
      }
    }
    lt.startListening();
  }
}

class My_interpreter2: public Cetaguard_message_interpreter{
  void button_msg(Cetaguard_msg_contents contents, Cetaguard_index idx){
  }
  void pairing_without_secret_msg(Cetaguard_index idx){
  }
  void pairing_with_secret_msg(Cetaguard_index idx){
  }
};

My_interpreter2 interpreter2;


void benchmark(){
  Cetaguard_status ret;

  Cetaguard_transmitter txTest;
  uint32_t cycles_start, cycles;
  cycles_start = ESP.getCycleCount();
  Cetaguard_receiver rxTest(&interpreter2);
  cycles = ESP.getCycleCount() - cycles_start;
  Serial.printf("Creating rx: %u\n", cycles);

  cycles_start = ESP.getCycleCount();
  uint32_t id_out;
  ret = rxTest.add_pending_transmitter(&id_out);
  cycles = ESP.getCycleCount() - cycles_start;
  Serial.printf("adding pending tx: %u\n", cycles);
  Serial.printf("\t keygen: %u\n", debug_data[1] - debug_data[0]);
  Serial.printf("\t random: %u\n", debug_data[2] - debug_data[1]);
  Serial.printf("\t hash: %u\n", debug_data[3] - debug_data[2]);
  if (ret != CETAGUARD_OK){
    Serial.println("error");
    return;
  }

  cycles_start = ESP.getCycleCount();
  ret = txTest.add_receiver(rxTest.pending[0].pub_key, rxTest.pending[0].pairing_secret);
  if (ret != CETAGUARD_OK){
    Serial.println("error");
    return;
  }
  
  cycles = ESP.getCycleCount() - cycles_start;
  Serial.printf("Adding rx: %u\n", cycles);

  uint8_t msg[64];
  size_t size;
  
  cycles_start = ESP.getCycleCount();
  ret = txTest.prepare_pairing_msg(0, msg, &size);
  cycles = ESP.getCycleCount() - cycles_start;
  Serial.printf("Prepairing pairing msg: %u\n", cycles);
  if (ret != CETAGUARD_OK){
    Serial.println("error");
    return;
  }

  cycles_start = ESP.getCycleCount();
  ret = rxTest.interpret_message(msg, size);
  cycles = ESP.getCycleCount() - cycles_start;
  Serial.printf("Interpreting pairing msg: %u\n", cycles);
  if (ret != CETAGUARD_OK){
    Serial.println("error");
    return;
  }
  
  cycles_start = ESP.getCycleCount();
  ret = txTest.finish_pairing(0);
  cycles = ESP.getCycleCount() - cycles_start;
  Serial.printf("Finish pairing: %u\n", cycles);
  if (ret != CETAGUARD_OK){
    Serial.print(ret);
    Serial.println("error");
    return;
  }
  
  Cetaguard_msg_contents c_in;
  cycles_start = ESP.getCycleCount();
  ret = txTest.prepare_button_msg(0, &c_in, msg, &size);
  cycles = ESP.getCycleCount() - cycles_start;
  Serial.printf("Preparing button msg: %u\n", cycles);
  Serial.printf("\t prepare: %u\n", debug_data[1] - debug_data[0]);
  Serial.printf("\t hash: %u\n", debug_data[2] - debug_data[1]);
  Serial.printf("\t encrypt: %u\n", debug_data[3] - debug_data[2]);
  if (ret != CETAGUARD_OK){
    Serial.print(ret);
    Serial.println("error");
    return;
  }
  
  cycles_start = ESP.getCycleCount();
  ret = rxTest.interpret_message(msg, size);
  cycles = ESP.getCycleCount() - cycles_start;
  Serial.printf("Interpreting button msg: %u\n", cycles);
  Serial.printf("\t decrypt: %u\n", debug_data[1] - debug_data[0]);
  if (ret != CETAGUARD_OK){
    Serial.print(ret);
    Serial.println("error");
    return;
  }
  
}
