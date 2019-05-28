#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServerSecure.h>
#include <DNSServer.h>
#include <FS.h>
#include "EndUserSetup.h"

ESP8266WebServer* p_webServer = NULL;
static const char tab[] = "\t";

void handleWifiScan(){
  int n = WiFi.scanNetworks();
  String response;
  for (int i = 0; i < n; i++){
    response += WiFi.SSID(i) + tab;
    response += String(WiFi.RSSI(i)) + tab;
    response += WiFi.BSSIDstr(i) + tab;
    response += String(WiFi.channel(i)) + tab;
    response += String(WiFi.encryptionType(i)) + "\r\n";
  }
  response += "\r\n";
  p_webServer->send(200, "text/plain", response);
}

//TODO: Handle unsecured networks. Or maybe not. People need to learn.
void handleWifiData(){
  String ssid = "";
  String pass = "";
  
  for (uint8_t i=0; i<p_webServer->args(); i++){
    if (p_webServer->argName(i) == "ssid"){
      ssid = p_webServer->arg(i);
      Serial.println(p_webServer->arg(i));
    }
    else if (p_webServer->argName(i) == "pass") {
      pass = p_webServer->arg(i);
      Serial.println(p_webServer->arg(i));
    }
  }
  
  if (ssid == "" || pass == "") {
    p_webServer->send(500, "text/plain", F("Niekompletne dane"));
    return;
  }
  
  WiFi.begin(ssid.c_str(), pass.c_str());

  int timeout = 10;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (!(--timeout)) {
      break;
    }
  }

  String ip_str = "";//WiFi.localIP();
  if (WiFi.status() != WL_CONNECTED){
    Serial.printf("status code %d\n", WiFi.status());
    Serial.println(ip_str);
    p_webServer->send(200, "text/plain", F("Nie polaczono. Sprawdz dane"));
  }
  else {
    wifi_station_set_auto_connect(1);
    Serial.println(ip_str);
    p_webServer->send(200, "text/plain", ip_str);
  }
}

void performEndUserSetup(void){
  ESP8266WebServer webServer(80);
  DNSServer dnsServer;
  IPAddress apIP(10, 10, 10, 1);    // Private network for server
  WiFi.disconnect();
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP("ESP_config", "kopytko1");
  dnsServer.start(53, "*", apIP);

  p_webServer = &webServer;
  webServer.on("/send_data", HTTP_POST, handleWifiData);
  webServer.on("/scan_networks", HTTP_GET, handleWifiScan);
  webServer.onNotFound([]() {
      Serial.println(F("Request"));
      File file = SPIFFS.open("/setup.htm", "r");
      size_t sent = p_webServer->streamFile(file, "text/html");
      file.close();
  });
  
  webServer.begin(); 

  while (1){
    dnsServer.processNextRequest();
    webServer.handleClient();
    yield();
  }
}

