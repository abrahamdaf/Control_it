#include "WiFi.h"
#include "AsyncUDP.h"

const char * ssid = "abradeal";
const char * password = "control_it";

AsyncUDP udp;

void setup()
{
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.println("WiFi Failed");
        while(1) {
            delay(1000);
        }
    }
    if(udp.connect(IPAddress(192,168,1,100), 1234)) {
        Serial.println("UDP connected");
        udp.onPacket([](AsyncUDPPacket packet) {
            Serial.print("UDP Packet Type: ");
            Serial.print(packet.isBroadcast()?"Broadcast":packet.isMulticast()?"Multicast":"Unicast");
            Serial.print(", From: ");
            Serial.print(packet.remoteIP());
            Serial.print(":");
            Serial.print(packet.remotePort());
            Serial.print(", To: ");
            Serial.print(packet.localIP());
            Serial.print(":");
            Serial.print(packet.localPort());
            Serial.print(", Length: ");
            Serial.print(packet.length());
            Serial.print(", Data: ");
            Serial.write(packet.data(), packet.length());
            Serial.println();
            //reply to the client
            packet.printf("Got %u bytes of data", packet.length());
        });
        //Send unicast
        udp.print("Hello Server!");
    }
}

void loop(){
    //Send broadcast on port 1234
    udp.broadcastTo("Si caben?", 1234);
    delay(500);
//    for(int i = 0; i < 100; i++){
//      char sendNudes[1] = {48+i}; 
//      udp.print(i);
//      delay(1000);
//    }
}
