/*************************************************** 

 ****************************************************/

#include "Arduino.h"
#ifdef __AVR__
  #include <SoftwareSerial.h>
#endif

#define GPS_DEBUG

enum NavMode { Pedestrian, Automotive, Sea, Airborne };
enum DataRate { F1Hz, F2Hz, F3_33Hz, F4Hz };

struct UBX_msg {
  byte class_byte;
  byte id_byte;
  uint16_t payload_length;
  byte* msg;
  byte CK_A;
  byte CK_B;
  };

struct Time_UTC {
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  bool valid = false;
};

struct Location {
  double longitude = 0.0;
  double latitude = 0.0;
  bool valid = false;
};


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

#define UBX_SYNC_1 0xB5
#define UBX_SYNC_2 0x62

// definition of UBX class IDs
// source: U-blox7 V14 Receiver Description Protocol page 88 https://www.u-blox.com/sites/default/files/products/documents/u-blox7-V14_ReceiverDescriptionProtocolSpec_%28GPS.G7-SW-12001%29_Public.pdf
#define NAV_CLASS 0x01 // Navigation Results: Position, Speed, Time, Acc, Heading, DOP, SVs used
#define RXM_CLASS 0x02 // Receiver Manager Messages: Satellite Status, RTC Status
#define INF_CLASS 0x04 // Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
#define ACK_CLASS 0x05 // Ack/Nack Messages: as replies to CFG Input Messages
#define CFG_CLASS 0x06 // Configuration Input Messages: Set Dynamic Model, Set DOP Mask, Set Baud Rate, etc
#define MON_CLASS 0x0A // Monitoring Messages: Comunication Status, CPU Load, Stack Usage, Task Status
#define AID_CLASS 0x0B // AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
#define TIM_CLASS 0x0D // Timing Messages: Time Pulse Output, Timemark Results
#define LOG_CLASS 0x21 // Logging Messages: Log creation, deletion, info and retrieval

class VMA430_GPS {
 public:
#ifdef __AVR__
  VMA430_GPS(SoftwareSerial *);
#endif
  VMA430_GPS(HardwareSerial *);

  void begin(int32_t baudrate);
  void getconfig(void);
  bool getUBX_packet(void);
  void setUBXNav(void);
  bool parse_ubx_data(void);
  
  NavMode NavigationMode = Pedestrian;
  DataRate DataRefreshRate = F4Hz;
 
  bool GLLSentence;
  bool GSASentence;
  bool GSVSentence;
  bool RMCSentence;
  bool VTGSentence;

  byte buffer_msg[60];

  Time_UTC utc_time;
  Location location;

  


  
 private: 
	byte* generateConfiguration(void);
	void sendConfiguration(void);
	void receive(void);
	void calcChecksum(byte *, byte);
	void sendUBX(byte *, byte);
  bool parse_ubx_nav_data(void);

  bool parse_nav_timeutc(void);
  bool parse_nav_pos(void);
  
	byte getUBX_ACK(byte *);
	
	long portRate;
	Stream *stream = NULL;
	HardwareSerial *hwSerial = NULL; 
  UBX_msg latest_msg;
#ifdef __AVR__	
	SoftwareSerial *swSerial = NULL;
#endif
};
