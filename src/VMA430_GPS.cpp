/*************************************************** 

 ****************************************************/

#include "VMA430_GPS.h"
#ifdef __AVR__
    #include <util/delay.h>
    #include <SoftwareSerial.h>
#endif

//#define GPS_DEBUG

#ifdef __AVR__
VMA430_GPS::VMA430_GPS(SoftwareSerial *ss) 
{
	swSerial 	= ss;
	stream 		= ss;
}
#endif

VMA430_GPS::VMA430_GPS(HardwareSerial *ss) 
{
	hwSerial 	= ss;
	stream 		= ss;
}

void VMA430_GPS::begin(int32_t baudrate) 
{
	switch (baudrate) 
	{
		case 4800:
			portRate = PORT_RATE_4800;
			break;
		case 9600:
			portRate = PORT_RATE_9600;
			break;
		case 19200:
			portRate = PORT_RATE_19200;
			break;
		case 57600:
			portRate = PORT_RATE_57600;
			break;
		case 115200:
			portRate = PORT_RATE_115200;
			break;
		case 230400:
			portRate = PORT_RATE_230400;
			break;
		default:
			return;
			break;		
	}
	
	if (hwSerial) hwSerial->begin(baudrate);

#ifdef __AVR__
	if (swSerial) swSerial->begin(baudrate);
#endif 

	sendConfiguration();
}

byte* VMA430_GPS::generateConfiguration()
{
	byte* settings = new byte[10];
	
	switch (NavigationMode) 
	{
		case Pedestrian:
			settings[0] = NAV_MODE_PEDESTRIAN;
			break;
		case Automotive:
			settings[0] = NAV_MODE_AUTOMOTIVE;
			break;
		case Sea:
			settings[0] = NAV_MODE_SEA;
			break;
		case Airborne:
			settings[0] = NAV_MODE_AIRBORNE;
			break;
	}
	
	switch (DataRefreshRate) 
	{
		case F1Hz:
			settings[1] = highByte(DATA_RATE_1HZ);
			settings[1] = lowByte(DATA_RATE_1HZ);
			break;
		case F2Hz:
			settings[1] = highByte(DATA_RATE_2HZ);
			settings[1] = lowByte(DATA_RATE_2HZ);
			break;
		case F3_33Hz:
			settings[1] = highByte(DATA_RATE_3_33HZ);
			settings[1] = lowByte(DATA_RATE_3_33HZ);
			break;
		case F4Hz:
			settings[1] = highByte(DATA_RATE_4HZ);
			settings[1] = lowByte(DATA_RATE_4HZ);
			break;
	}
	
	settings[3] = portRate >> 16;
	settings[4] = portRate >> 8;
	settings[5] = portRate & 0xFF;
	settings[6] = GLLSentence ? 0x01 : 0x00;
	settings[7] = GSVSentence ? 0x01 : 0x00;
	settings[8] = RMCSentence ? 0x01 : 0x00;
	settings[9] = VTGSentence ? 0x01 : 0x00;
	
	return settings;
}

void VMA430_GPS::sendConfiguration(void) 
{
	byte* settings = generateConfiguration();
	byte gpsSetSuccess = 0;
	
#ifdef GPS_DEBUG
	Serial.println("Configuring u-Blox GPS initial state...");
#endif
	
	//Generate the configuration string for Navigation Mode
	byte setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, *settings, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	calcChecksum(&setNav[2], sizeof(setNav) - 4);
	
	//Generate the configuration string for Data Rate
	byte setDataRate[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, settings[1], settings[2], 0x01, 0x00, 0x01, 0x00, 0x00, 0x00};
	calcChecksum(&setDataRate[2], sizeof(setDataRate) - 4);

	//Generate the configuration string for Baud Rate
	byte setPortRate[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, settings[3], settings[4], settings[5], 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	calcChecksum(&setPortRate[2], sizeof(setPortRate) - 4);

	byte setGLL[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B};
	byte setGSA[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32};
	byte setGSV[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39};
	byte setRMC[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40};
	byte setVTG[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46};
	
	boolean gpsStatus[] = {false, false, false, false, false, false, false};

	//  Navigation mode 
	while(gpsSetSuccess < 3)
	{
#ifdef GPS_DEBUG
		Serial.println("Setting Navigation mode...");
#endif		
		sendUBX(&setNav[0], sizeof(setNav));  //Send UBX Packet
		gpsSetSuccess += getUBX_ACK(&setNav[2]); //Passes Class ID and Message ID to the ACK Receive function
		if (gpsSetSuccess == 5) {
		  gpsSetSuccess -= 4;
		  delay(1500);
		  byte lowerPortRate[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA2, 0xB5};
		  sendUBX(lowerPortRate, sizeof(lowerPortRate));
		  delay(2000);
		}
		
		if (gpsSetSuccess == 6) gpsSetSuccess -= 4;
		if (gpsSetSuccess == 10) gpsStatus[0] = true;
	}
	
	if (gpsSetSuccess == 3) Serial.println("Navigation mode configuration failed.");
	gpsSetSuccess = 0;
  
	// Data update rate
	while(gpsSetSuccess < 3) 
	{
#ifdef GPS_DEBUG
		Serial.print("Setting Data Update Rate...");
#endif

		sendUBX(&setDataRate[0], sizeof(setDataRate));  //Send UBX Packet
		gpsSetSuccess += getUBX_ACK(&setDataRate[2]); //Passes Class ID and Message ID to the ACK Receive function
		if (gpsSetSuccess == 10) gpsStatus[1] = true;
		if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
	}
	
	if (gpsSetSuccess == 3) Serial.println("Data update mode configuration failed.");
	gpsSetSuccess = 0;

	// NMEA GLL messages
	while(gpsSetSuccess < 3 && settings[6] == 0x00) 
	{
#ifdef GPS_DEBUG		
		Serial.print("Deactivating NMEA GLL Messages");
#endif
		sendUBX(setGLL, sizeof(setGLL));
		gpsSetSuccess += getUBX_ACK(&setGLL[2]);
		if (gpsSetSuccess == 10) gpsStatus[2] = true;
		if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
	}
	
	if (gpsSetSuccess == 3) Serial.println("NMEA GLL Message Deactivation Failed!");
	gpsSetSuccess = 0;

	// NMEA GSA messages
	while(gpsSetSuccess < 3 && settings[7] == 0x00) 
	{
#ifdef GPS_DEBUG		
		Serial.print("Deactivating NMEA GSA Messages");
#endif		
		sendUBX(setGSA, sizeof(setGSA));
		gpsSetSuccess += getUBX_ACK(&setGSA[2]);
		if (gpsSetSuccess == 10) gpsStatus[3] = true;
		if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
	}
	  
	if (gpsSetSuccess == 3) Serial.println("NMEA GSA Message Deactivation Failed!");
	gpsSetSuccess = 0;

	// NMEA GSV messages
	while(gpsSetSuccess < 3 && settings[8] == 0x00) 
	{
#ifdef GPS_DEBUG
		Serial.print("Deactivating NMEA GSV Messages ");
#endif
		sendUBX(setGSV, sizeof(setGSV));
		gpsSetSuccess += getUBX_ACK(&setGSV[2]);
		
		if (gpsSetSuccess == 10) gpsStatus[4] = true;
		if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
	}
  
	if (gpsSetSuccess == 3) Serial.println("NMEA GSV Message Deactivation Failed!");
	gpsSetSuccess = 0;

	// NMEA RMC messages
	while(gpsSetSuccess < 3 && settings[9] == 0x00) 
	{
#ifdef GPS_DEBUG		
		Serial.print("Deactivating NMEA RMC Messages");
#endif	
		sendUBX(setRMC, sizeof(setRMC));
		gpsSetSuccess += getUBX_ACK(&setRMC[2]);
		if (gpsSetSuccess == 10) gpsStatus[5] = true;
		if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
	}
	
	if (gpsSetSuccess == 3) Serial.println("NMEA RMC Message Deactivation Failed!");
	gpsSetSuccess = 0;

	// NMEA VTG messages
	while(gpsSetSuccess < 3 && settings[10] == 0x00) 
	{
#ifdef GPS_DEBUG
		Serial.print("Deactivating NMEA VTG Messages ");
#endif
		sendUBX(setVTG, sizeof(setVTG));
		gpsSetSuccess += getUBX_ACK(&setVTG[2]);
		if (gpsSetSuccess == 10) gpsStatus[6] = true;
		if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
	}
  
	if (gpsSetSuccess == 3) Serial.println("NMEA VTG Message Deactivation Failed!");
	gpsSetSuccess = 0;
	
	if (settings[4] != 0x25) {
		Serial.print("Setting Port Baud Rate... ");
		sendUBX(&setPortRate[0], sizeof(setPortRate));
		Serial.println("Success!");
		delay(500);
	}
	
	delete[] settings;
}

void VMA430_GPS::sendUBX(byte *UBXmsg, byte msgLength) 
{
  for (int i = 0; i < msgLength; i++) 
  {
	stream->write(UBXmsg[i]);
    stream->flush();
  }
  
  stream->println();
  stream->flush();
}

byte VMA430_GPS::getUBX_ACK(byte *msgID) 
{
  byte CK_A = 0, CK_B = 0;
  byte incoming_char;
  boolean headerReceived = false;
  unsigned long ackWait = millis();
  byte ackPacket[10] = {0xB5, 0x62, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  int i = 0;
  while (1) {
    if (stream->available()) {
      incoming_char = stream->read();
      if (incoming_char == ackPacket[i]) {
        i++;
      }
      else if (i > 2) {
        ackPacket[i] = incoming_char;
        i++;
      }
    }
    if (i > 9) break;
    if ((millis() - ackWait) > 1500) {
      Serial.println("ACK Timeout");
      return 5;
    }
    if (i == 4 && ackPacket[3] == 0x00) {
      Serial.println("NAK Received");
      return 1;
    }
  }

  for (i = 2; i < 8 ;i++) {
  CK_A = CK_A + ackPacket[i];
  CK_B = CK_B + CK_A;
  }
  if (msgID[0] == ackPacket[6] && msgID[1] == ackPacket[7] && CK_A == ackPacket[8] && CK_B == ackPacket[9]) {
    Serial.println("Success!");
    Serial.print("ACK Received! ");
    //printHex(ackPacket, sizeof(ackPacket));
    return 10;
        }
  else {
    Serial.print("ACK Checksum Failure: ");
    //printHex(ackPacket, sizeof(ackPacket));
    delay(1000);
    return 1;
  }
}


void VMA430_GPS::calcChecksum(byte *checksumPayload, byte payloadSize) {
  byte CK_A = 0, CK_B = 0;
  for (int i = 0; i < payloadSize ;i++) {
    CK_A = CK_A + *checksumPayload;
    CK_B = CK_B + CK_A;
    checksumPayload++;
  }
  *checksumPayload = CK_A;
  checksumPayload++;
  *checksumPayload = CK_B;
}