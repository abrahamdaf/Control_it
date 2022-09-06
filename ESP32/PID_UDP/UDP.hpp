// UDP variables
WiFiUDP UDP;

unsigned int localPort = 6666;
unsigned int remotePort = 8889;
char packetBuffer[1024]; //buffer to hold incoming packet,
