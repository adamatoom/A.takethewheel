#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <SoftwareSerial.h>
#include <PID_v1.h>

#define MYPORT_TX 14
#define MYPORT_RX 12

#define WIFI_SSID "Internet"
#define WIFI_PASS "adam1999"
//#define WIFI_SSID "ZWnet"
//#define WIFI_PASS "Zolom.123"
#define UDP_PORT 4210

WiFiUDP UDP;
SoftwareSerial mySerial;

char buff[2];
char packet[255];
byte type = 0;
byte count;
int ang[5] = {0, 0, 0, 0, 0};
int rang[5] = {0, 0, 0, 0, 0};
float p = 7.0 / 400.0;
float i = 3.0 / 200000.0;
float d = 0;
float P = 0;
unsigned long time1  = millis();
float I = 0;
float D = 0;
float dt = 1000000;
char reply[] = "Packet received!";
int counter = 0;
int str = 127;
byte output = 127;
int str0 = 127;
double Setpoint, Input, Output;
byte pid = 0;

PID myPID(&Input, &Output, &Setpoint, 7.0 / 400.0, 3.0 / 2000.0, 0, DIRECT);
void setup()
{
  // Set your Static IP address
  //  IPAddress local_IP(192, 168, 20, 63);
  //  // Set your Gateway IP address
  //  IPAddress gateway(192, 168, 20, 227);
  //
  //  IPAddress subnet(255, 255, 255, 0);
  // Setup serial port
  Serial.begin(115200);
  Serial1.begin(115200);
  mySerial.begin(38400, SWSERIAL_8N1, MYPORT_RX, MYPORT_TX, false);
  if (!mySerial)
  {
    Serial.println("NO");
  }
  //
  //  Serial2.begin(115200);
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

  //  Connected to WiFi
  Serial.println();
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());

  // Begin listening to UDP port
  UDP.begin(UDP_PORT);
  //  Serial.print("Listening on UDP port ");
  //  Serial.println(UDP_PORT);
  analogWriteRange(255);
  analogWrite(4, 127);
}

void loop()
{
  // put your main code here, to run repeatedly:

  int packetSize = UDP.parsePacket();
  if (packetSize)
  {
    Serial.println("UDP1");
    //    Serial.print("Received packet! Size: ");
    //    Serial.println(packetSize);
    int len = UDP.read(packet, 255);
    if (len > 0)
    {
      Serial.println("UDP2");
      //      packet[len] = '\0';
      type = packet[0];
      switch (type)
      {
      case (byte)0:
        //code to be executed;
        rang[3] = rang[4];
        rang[2] = rang[3];
        rang[1] = rang[2];
        rang[0] = rang[1];
        rang[4] = (packet[2] | packet[1] << 8) - 4000;
        Serial.println("rang");
        Setpoint = (double)rang[4];
        pid = 1;
        
        // Serial.println(rang[4]);
        break;
      case (byte)1:
        //code to be executed
        output = packet[1];
        // Serial.println((int)output);
        Serial.println("torque");
        pid = 0;
        break;
      case (byte)3:
        //code to be executed
        p = float(packet[1]) / 400.0;
        i = float(packet[2]) / 200000.0;
        d = float(packet[3]) / 2000.0;
        Serial.println("PID");
        break;
      default:
        Serial.println("Invalid Type");
        //code to be executed if all cases are not matched
      }
      //    str = packet[1];
    }
    //    Serial.print("Packet received: ");
    //    for (int i=0;i<len;i++){

    //    }

    //    UDP.beginPacket(UDP.remoteIP(), UDP.remotePort());
    //    UDP.write(reply);
    //    UDP.endPacket();
  }
  count = 0;
  //    Serial.println("Yes");
  if (mySerial.available() > 1)
  {
    //      buff[count] = mySerial.read();
    //      count ++;
    for (int i = 0; i < 2; i++)
    {
      buff[i] = mySerial.read();
    }
    // Serial.println(" ");
    // Serial.println((int)buff[0]);
    // Serial.println((int)buff[1]);
    // Serial.println(ang[4] - 5000);
    ang[3] = ang[4];
    ang[2] = ang[3];
    ang[1] = ang[2];
    ang[0] = ang[1];
    ang[4] = buff[0] << 8 | buff[1];
    if (counter % 10)
    {
      Serial.println("rec rang");
      Serial.println(ang[4]);
      Setpoint = (double)ang[4];
    }
  }
  if (type == 0)
  {
    output = (byte)clamp((float)0, (float)255, PID() + (float)127);
  }
  if (type == 0 || type == 1)
  {
    str = (int)clamp(str0 - 5, str0 + 5, (int)output);
  }
  Serial1.write((byte)str);
  Serial.println((int)str);
  str0 = str;
}

byte clamp(byte lower, byte higher, byte input)
{
  if (input < lower)
  {
    input = lower;
  }
  if (input > higher)
  {
    input = higher;
  }
  return input;
}
int clamp(int lower, int higher, int input)
{
  if (input < lower)
  {
    input = lower;
  }
  if (input > higher)
  {
    input = higher;
  }
  return input;
}
float clamp(float lower, float higher, float input)
{
  if (input < lower)
  {
    input = lower;
  }
  if (input > higher)
  {
    input = higher;
  }
  return input;
}
float PID()
{
  dt = (float)(millis() - time1);
  P = clamp(-100.0, 100.0, p * ((float)rang[4] - (float)ang[4]));
  I = clamp(-27.0, 27.0, I + (((float)rang[4] - (float)ang[4]) + (float)rang[0] - (float)ang[0]) * i * dt / 2);
  D = clamp(-30.0, 30.0, (((float)rang[4] - (float)ang[4]) - ((float)rang[0] - (float)ang[0])) * d / dt);

  return P + I + D;
}
