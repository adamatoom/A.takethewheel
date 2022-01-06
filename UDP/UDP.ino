#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <SoftwareSerial.h>
// Set WiFi credentials
#define WIFI_SSID "Internet"
#define WIFI_PASS "adam1999"
//#define WIFI_SSID "ZWnet"
//#define WIFI_PASS "Zolom.123"
#define UDP_PORT 4210
   
WiFiUDP UDP;


char packet[255];
char reply[] = "Packet received!";

      byte str =127;
void setup() {
  // Set your Static IP address
//  IPAddress local_IP(192, 168, 20, 63);
//  // Set your Gateway IP address
//  IPAddress gateway(192, 168, 20, 227);
//  
//  IPAddress subnet(255, 255, 255, 0);
  // Setup serial port
  Serial.begin(115200);
  Serial1.begin(115200);
//  Serial.println();

  // Configures static IP address
//  if (!WiFi.config(local_IP, gateway, subnet)) {
//    Serial.println("STA Failed to configure");
//  }
   
  // Begin WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASS);
   
  // Connecting to WiFi...
//  Serial.print("Connecting to ");
//  Serial.print(WIFI_SSID);
  // Loop continuously while WiFi is not connected
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
//    Serial.print(".");
  }
   
  // Connected to WiFi
//  Serial.println();
//  Serial.print("Connected! IP address: ");
//  Serial.println(WiFi.localIP());
  
  // Begin listening to UDP port
  UDP.begin(UDP_PORT);
//  Serial.print("Listening on UDP port ");
//  Serial.println(UDP_PORT);
   analogWriteRange(255);
      analogWrite(4, 127);
}
   
void loop() {  
  // put your main code here, to run repeatedly:
   
  int packetSize = UDP.parsePacket();
  if (packetSize) {
//    Serial.print("Received packet! Size: ");
//    Serial.println(packetSize); 
  int len = UDP.read(packet, 255);
  if (len > 0)
  {
//      packet[len] = '\0';
    type = packet[0];
    str = packet[1];
  }
//    Serial.print("Packet received: ");
//    for (int i=0;i<len;i++){
  
    analogWrite(4, str);
//    }
    Serial.println(str);
    Serial1.write(str);
//    UDP.beginPacket(UDP.remoteIP(), UDP.remotePort());
//    UDP.write(reply);
//    UDP.endPacket();
  }
}
