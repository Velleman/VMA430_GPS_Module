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
	swSerial = ss;
	stream = ss;
}
#endif

VMA430_GPS::VMA430_GPS(HardwareSerial *ss)
{
	hwSerial = ss;
	stream = ss;
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

	if (hwSerial)
		hwSerial->begin(baudrate);

#ifdef __AVR__
	if (swSerial)
		swSerial->begin(baudrate);
#endif

	sendConfiguration();
}

byte *VMA430_GPS::generateConfiguration()
{
	byte *settings = new byte[10];

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
	byte *settings = generateConfiguration();
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
	while (gpsSetSuccess < 3)
	{
#ifdef GPS_DEBUG
		Serial.println("Setting Navigation mode...");
#endif
		sendUBX(&setNav[0], sizeof(setNav));	 //Send UBX Packet
		gpsSetSuccess += getUBX_ACK(&setNav[2]); //Passes Class ID and Message ID to the ACK Receive function
		if (gpsSetSuccess == 5)
		{
			gpsSetSuccess -= 4;
			delay(1500);
			byte lowerPortRate[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA2, 0xB5};
			sendUBX(lowerPortRate, sizeof(lowerPortRate));
			delay(2000);
		}

		if (gpsSetSuccess == 6)
			gpsSetSuccess -= 4;
		if (gpsSetSuccess == 10)
			gpsStatus[0] = true;
	}

	if (gpsSetSuccess == 3)
		Serial.println("Navigation mode configuration failed.");
	gpsSetSuccess = 0;

	// Data update rate
	/*while (gpsSetSuccess < 3)
	{
#ifdef GPS_DEBUG
		Serial.print("Setting Data Update Rate...");
#endif

		sendUBX(&setDataRate[0], sizeof(setDataRate)); //Send UBX Packet
		gpsSetSuccess += getUBX_ACK(&setDataRate[2]);  //Passes Class ID and Message ID to the ACK Receive function
		if (gpsSetSuccess == 10)
			gpsStatus[1] = true;
		if (gpsSetSuccess == 5 | gpsSetSuccess == 6)
			gpsSetSuccess -= 4;
	}

	if (gpsSetSuccess == 3)
		Serial.println("Data update mode configuration failed.");
	gpsSetSuccess = 0;

	// NMEA GLL messages
	while (gpsSetSuccess < 3 && settings[6] == 0x00)
	{
#ifdef GPS_DEBUG
		Serial.print("Deactivating NMEA GLL Messages");
#endif
		sendUBX(setGLL, sizeof(setGLL));
		gpsSetSuccess += getUBX_ACK(&setGLL[2]);
		if (gpsSetSuccess == 10)
			gpsStatus[2] = true;
		if (gpsSetSuccess == 5 | gpsSetSuccess == 6)
			gpsSetSuccess -= 4;
	}

	if (gpsSetSuccess == 3)
		Serial.println("NMEA GLL Message Deactivation Failed!");
	gpsSetSuccess = 0;

	// NMEA GSA messages
	while (gpsSetSuccess < 3 && settings[7] == 0x00)
	{
#ifdef GPS_DEBUG
		Serial.print("Deactivating NMEA GSA Messages");
#endif
		sendUBX(setGSA, sizeof(setGSA));
		gpsSetSuccess += getUBX_ACK(&setGSA[2]);
		if (gpsSetSuccess == 10)
			gpsStatus[3] = true;
		if (gpsSetSuccess == 5 | gpsSetSuccess == 6)
			gpsSetSuccess -= 4;
	}

	if (gpsSetSuccess == 3)
		Serial.println("NMEA GSA Message Deactivation Failed!");
	gpsSetSuccess = 0;

	// NMEA GSV messages
	while (gpsSetSuccess < 3 && settings[8] == 0x00)
	{
#ifdef GPS_DEBUG
		Serial.print("Deactivating NMEA GSV Messages ");
#endif
		sendUBX(setGSV, sizeof(setGSV));
		gpsSetSuccess += getUBX_ACK(&setGSV[2]);

		if (gpsSetSuccess == 10)
			gpsStatus[4] = true;
		if (gpsSetSuccess == 5 | gpsSetSuccess == 6)
			gpsSetSuccess -= 4;
	}

	if (gpsSetSuccess == 3)
		Serial.println("NMEA GSV Message Deactivation Failed!");
	gpsSetSuccess = 0;

	// NMEA RMC messages
	while (gpsSetSuccess < 3 && settings[9] == 0x00)
	{
#ifdef GPS_DEBUG
		Serial.print("Deactivating NMEA RMC Messages");
#endif
		sendUBX(setRMC, sizeof(setRMC));
		gpsSetSuccess += getUBX_ACK(&setRMC[2]);
		if (gpsSetSuccess == 10)
			gpsStatus[5] = true;
		if (gpsSetSuccess == 5 | gpsSetSuccess == 6)
			gpsSetSuccess -= 4;
	}

	if (gpsSetSuccess == 3)
		Serial.println("NMEA RMC Message Deactivation Failed!");
	gpsSetSuccess = 0;

	// NMEA VTG messages
	while (gpsSetSuccess < 3 && settings[10] == 0x00)
	{
#ifdef GPS_DEBUG
		Serial.print("Deactivating NMEA VTG Messages ");
#endif
		sendUBX(setVTG, sizeof(setVTG));
		gpsSetSuccess += getUBX_ACK(&setVTG[2]);
		if (gpsSetSuccess == 10)
			gpsStatus[6] = true;
		if (gpsSetSuccess == 5 | gpsSetSuccess == 6)
			gpsSetSuccess -= 4;
	}

	if (gpsSetSuccess == 3)
		Serial.println("NMEA VTG Message Deactivation Failed!");
	gpsSetSuccess = 0;
*/
	if (settings[4] != 0x25)
	{
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

void VMA430_GPS::setUBXNav(void)
{
	byte setNAVUBX[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x21, 0x01, 0x00, 0x00};
	byte setNAVUBX_pos[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x00, 0x00};
	Serial.println("Enabling UBX time NAV data");
	byte CK_A = 0, CK_B = 0;
	for (int i = 0; i < 7; i++)
	{
		CK_A = CK_A + setNAVUBX[i + 2];
		CK_B = CK_B + CK_A;
	}
	setNAVUBX[9] = CK_A;
	setNAVUBX[10] = CK_B;
	sendUBX(&setNAVUBX[0], sizeof(setNAVUBX));
	getUBX_ACK(&setNAVUBX[2]);

	Serial.println("Enabling UBX position NAV data");
	CK_A = 0, CK_B = 0;
	for (int i = 0; i < 7; i++)
	{
		CK_A = CK_A + setNAVUBX_pos[i + 2];
		CK_B = CK_B + CK_A;
	}
	setNAVUBX_pos[9] = CK_A;
	setNAVUBX_pos[10] = CK_B;
	sendUBX(&setNAVUBX_pos[0], sizeof(setNAVUBX_pos));
	getUBX_ACK(&setNAVUBX_pos[2]);
}
void VMA430_GPS::getconfig(void)
{
	byte get_cfg_message[8] = {UBX_SYNC_1, UBX_SYNC_2, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00};

	byte CK_A = 0, CK_B = 0;
	for (int i = 0; i < 4; i++)
	{
		CK_A = CK_A + get_cfg_message[i + 2];
		CK_B = CK_B + CK_A;
	}
	get_cfg_message[6] = CK_A;
	get_cfg_message[7] = CK_B;
	sendUBX(&get_cfg_message[0], sizeof(get_cfg_message));
}
bool VMA430_GPS::getUBX_packet(void)
{
	byte sync_chars[2] = {UBX_SYNC_1, UBX_SYNC_2};
	byte UBX_packet[4] = {UBX_SYNC_1, UBX_SYNC_2, 0x00, 0x00};
	byte incoming_char;
	byte class_byte_temp;
	byte id_byte_temp;
	byte length_bytes[2] = {0x00, 0x00};
	uint16_t payload_length = 0;
	int i = 0;
	byte CK_A = 0;
	byte CK_B = 0;
	int checksum_idx = 0;
	unsigned long ubxWait = millis();
	bool received_valid_ubx = false;
	//stream->flush();
	while (1)
	{
		//Serial.println("Alive");
		if (millis() - ubxWait > 1500)
		{
			Serial.println("TimeOut UBX packet!");
			//parse_nav_pos();
			break;
		}
		if (stream->available())
		{
			incoming_char = stream->read();
			//Serial.print((char)incoming_char);
			//Serial.print(" ");
			if (i < 2)
			{
				if (incoming_char == sync_chars[i])
				{
					i++;
				}
			}
			else if (i > 1)
			{
				//Serial.print("i: ");
				//Serial.println(i);
				switch (i)
				{
				case 2:
					//Serial.println("got class byte");
					class_byte_temp = incoming_char;
					break;
				case 3:
					//Serial.println("got id byte");
					id_byte_temp = incoming_char;
					break;
				case 4:
					//Serial.println("got length byte 1");
					length_bytes[0] = incoming_char;
					break;
				case 5:
					length_bytes[1] = incoming_char;
					payload_length = length_bytes[1] << 8 | length_bytes[0];

					//Serial.print("Expecting payload with length: ");
					//Serial.println(payload_length);

					break;

				default:
					break;
				}
				if ((i > 5) && (checksum_idx == 0))
				{
					if(payload_length > 40)
					{
						Serial.println("Invalid UBX packet length!");
						break;
					}
					if (i - 5 <= payload_length)
					{
						buffer_msg[i - 6] = incoming_char;
					}
					else
					{
						//Serial.println("Got start of checksum");
						checksum_idx = i;
					}
				}

				if (checksum_idx != 0)
				{
					if (i == checksum_idx)
					{
						CK_A = incoming_char;
					}
					else if (i == checksum_idx + 1)
					{
						CK_B = incoming_char;
						/*Serial.println("Complete msg: ");
						Serial.print(class_byte_temp, HEX);
						Serial.print(" ");
						Serial.print(id_byte_temp, HEX);
						Serial.print(" Payload length:");
						Serial.print(payload_length);
						Serial.print(" Payload: ");
						for (int j = 0; j < payload_length; j++)
						{
							Serial.print(buffer_msg[j], HEX);
							Serial.print(" ");
						}*/
						received_valid_ubx = true;

						latest_msg.class_byte = class_byte_temp;
						latest_msg.id_byte = id_byte_temp;
						latest_msg.payload_length = payload_length;
						latest_msg.msg = buffer_msg;
						latest_msg.CK_A = CK_A;
						latest_msg.CK_B = CK_B;

						break;
					}
				}
				i++;
			}
		}
	}
	return received_valid_ubx;
}

bool VMA430_GPS::parse_ubx_data(void)
{
	bool successfull_parsing = false;
	switch (latest_msg.class_byte)
	{
	case NAV_CLASS:
		successfull_parsing = parse_ubx_nav_data();
		break;

	default:
		break;
	}
	return successfull_parsing;
}

bool VMA430_GPS::parse_ubx_nav_data(void)
{
	bool successfull_parsing = false;
	switch (latest_msg.id_byte)
	{
	case 0x21:
		successfull_parsing = parse_nav_timeutc();
		break;
	case 0x02:
		successfull_parsing = parse_nav_pos();
		break;
	default:
		break;
	}
	return successfull_parsing;
}

bool VMA430_GPS::parse_nav_timeutc(void)
{
	bool successfull_parsing = false;
	byte* msg_data = latest_msg.msg;

	if(latest_msg.payload_length != 20)
	{
		return successfull_parsing;
	}
	utc_time.year = msg_data[12] | (msg_data[13]<<8);
	utc_time.month = msg_data[14];
	utc_time.day = msg_data[15];
	utc_time.hour = msg_data[16];
	utc_time.minute = msg_data[17];
	utc_time.second = msg_data[18];
	//Serial.println("Validaty time data:");
	//Serial.println(msg_data[19], HEX);
	if(msg_data[19] == 0x07)
	{
		utc_time.valid = true;
	}
	else
	{
		utc_time.valid = false;
	}
	successfull_parsing = true;
	return successfull_parsing;

}

bool VMA430_GPS::parse_nav_pos(void)
{
	bool successfull_parsing = false;
	
	int32_t temp_lon = 0, temp_lat = 0;
	double longitude = 0.0;
	double latitude = 0.0;
	uint32_t temp_val = 0;
	//byte test_arr[] = {0x00, 0x45, 0x62, 0x1D, 0x4B, 0xE0, 0x4F, 0x02, 0x4A, 0x34, 0x65, 0x1E, 0x39, 0x5A, 0x00, 0x00, 0x63, 0xA6, 0xFF, 0xFF, 0xD7, 0x39, 0x01, 0x00, 0x9F, 0xB9, 0x00, 0x00};
	byte* msg_data = latest_msg.msg;

	if(latest_msg.payload_length != 28)
	{
		return false;
	}

	temp_lon = extractSignedLong(4, msg_data); 
	longitude = (double)temp_lon*0.0000001;
	
	temp_lat = extractSignedLong(8, msg_data); 
	latitude = (double)temp_lat*0.0000001;

	location.latitude = latitude;
	location.longitude = longitude;
	return successfull_parsing;
}

byte VMA430_GPS::getUBX_ACK(byte *msgID)
{
	byte CK_A = 0, CK_B = 0;
	byte incoming_char;
	boolean headerReceived = false;
	unsigned long ackWait = millis();
	byte ackPacket[10] = {0xB5, 0x62, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	int i = 0;
	while (1)
	{
		if (stream->available())
		{
			incoming_char = stream->read();
			if (incoming_char == ackPacket[i])
			{
				i++;
			}
			else if (i > 2)
			{
				ackPacket[i] = incoming_char;
				i++;
			}
		}
		if (i > 9)
			break;
		if ((millis() - ackWait) > 1500)
		{
			Serial.println("ACK Timeout");
			return 5;
		}
		if (i == 4 && ackPacket[3] == 0x00)
		{
			Serial.println("NAK Received");
			return 1;
		}
	}

	for (i = 2; i < 8; i++)
	{
		CK_A = CK_A + ackPacket[i];
		CK_B = CK_B + CK_A;
	}
	if (msgID[0] == ackPacket[6] && msgID[1] == ackPacket[7] && CK_A == ackPacket[8] && CK_B == ackPacket[9])
	{
		Serial.println("Success!");
		Serial.print("ACK Received! ");
		//printHex(ackPacket, sizeof(ackPacket));
		return 10;
	}
	else
	{
		Serial.print("ACK Checksum Failure: ");
		//printHex(ackPacket, sizeof(ackPacket));
		delay(1000);
		return 1;
	}
}

uint32_t VMA430_GPS::extractLong(uint8_t spotToStart, byte* msg_data)
{
  uint32_t val = 0;
  val |= (uint32_t)msg_data[spotToStart + 0] << 8 * 0;
  val |= (uint32_t)msg_data[spotToStart + 1] << 8 * 1;
  val |= (uint32_t)msg_data[spotToStart + 2] << 8 * 2;
  val |= (uint32_t)msg_data[spotToStart + 3] << 8 * 3;
  return (val);
}

//Just so there is no ambiguity about whether a uint32_t will cast to a int32_t correctly...
int32_t VMA430_GPS::extractSignedLong(uint8_t spotToStart, byte* msg_data)
{
  union // Use a union to convert from uint32_t to int32_t
  {
      uint32_t unsignedLong;
      int32_t signedLong;
  } unsignedSigned;

  unsignedSigned.unsignedLong = extractLong(spotToStart, msg_data);
  return (unsignedSigned.signedLong);
}

void VMA430_GPS::calcChecksum(byte *checksumPayload, byte payloadSize)
{
	byte CK_A = 0, CK_B = 0;
	for (int i = 0; i < payloadSize; i++)
	{
		CK_A = CK_A + *checksumPayload;
		CK_B = CK_B + CK_A;
		checksumPayload++;
	}
	*checksumPayload = CK_A;
	checksumPayload++;
	*checksumPayload = CK_B;
}