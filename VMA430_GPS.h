/*************************************************** 

 ****************************************************/

#include "Arduino.h"
#ifdef __AVR__
  #include <SoftwareSerial.h>
#endif

#define GPS_DEBUG

enum NavMode { Pedestrian, Automotive, Sea, Airborne };
enum DataRate { F1Hz, F2Hz, F3_33Hz, F4Hz };

#define NAV_MODE_PEDESTRIAN 0x03
#define NAV_MODE_AUTOMOTIVE 0x04
#define NAV_MODE_SEA 0x05
#define NAV_MODE_AIRBORNE 0x06

#define DATA_RATE_1HZ 0xE803
#define DATA_RATE_2HZ 0xFA01
#define DATA_RATE_3_33HZ 0x2C01
#define DATA_RATE_4HZ 0xFA00

#define PORT_RATE_4800 0xC01200
#define PORT_RATE_9600 0x802500
#define PORT_RATE_19200 0x004B00
#define PORT_RATE_38400 0x009600
#define PORT_RATE_57600 0x00E100
#define PORT_RATE_115200 0x00C200
#define PORT_RATE_230400 0x008400

class VMA430_GPS {
 public:
#ifdef __AVR__
  VMA430_GPS(SoftwareSerial *);
#endif
  VMA430_GPS(HardwareSerial *);

  void begin(int32_t baudrate);
  
  NavMode NavigationMode = Pedestrian;
  DataRate DataRefreshRate = F4Hz;
 
  bool GLLSentence;
  bool GSASentence;
  bool GSVSentence;
  bool RMCSentence;
  bool VTGSentence;
  
 private: 
	byte* generateConfiguration(void);
	void sendConfiguration(void);
	void receive(void);
	void calcChecksum(byte *, byte);
	void sendUBX(byte *, byte);
	byte getUBX_ACK(byte *);
	
	long portRate;
	Stream *stream = NULL;
	HardwareSerial *hwSerial = NULL; 
#ifdef __AVR__	
	SoftwareSerial *swSerial = NULL;
#endif
};
