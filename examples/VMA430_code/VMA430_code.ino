/* 
  @@@@@@@@@@@@@@@@@@@@@@                                                                                                                               
  @@@@@@@@@@@@@@@@@@@@@@             @@@.    @@@    @@@.     @@@@    @@@@        @@@@@@@@@        @@@@@@@@@@@@@.     .@@@@@@@@@@@@@        @@@@@@@@@  
  @@@@@%%@@@%%@@@%%@@@@@             @@@@    @@@    @@@@     @@@@    @@@@       @@@@@@@@@@        @@@@@@@@@@@@@@     @@@@@@@@@@@@@@        @@@@@@@@@  
  @@@@@  @@@  @@@  @@@@@             @@@@    @@@    @@@@     @@@@    @@@@       @@@@   @@@@        @@@@@   @@@@@       @@@@@   @@@@       @@@@   @@@@ 
  @@@@@            @@@@@             @@@@   @@@@    @@@@     @@@@@@@@@@@@       @@@@   @@@@        @@@@@   @@@@@       @@@@@   @@@@       @@@@   @@@@ 
  @@@@@   @    @   @@@@@             @@@@   @@@@    @@@@     @@@@@@@@@@@@       @@@@@@@@@@@        @@@@@   @@@@@       @@@@@   @@@@       @@@@@@@@@@@ 
  @@@@@            @@@@@             @@@@###@@@@@##@@@@@     @@@@    @@@@      @@@@@@@@@@@@       #@@@@@###@@@@@     ##@@@@@###@@@@       @@@@@@@@@@@
  @@@@@@@@@@@@@@@@@@@@@@             @@@@@@@@@@@@@@@@@@@     @@@@    @@@@      @@@@    @@@@@      @@@@@@@@@@@@@@     @@@@@@@@@@@@@@      @@@@     @@@@
  @@@@@@@@@@@@@@@@@@@@@@

  Whadda WPI430 GPS MODULE U-BLOX NEO-7M example

  The example program will first set-up the GPS-module by setting it's communication protocols. 
  It will enable the module to send out UBX messages next to the standard NMEA messages that it normally outputs.
  The program will then decode these UBX messages, and show the parsed output. 
  The data that is sent out includes location data (latitude & longitude) and UTC time (hours, minutes & seconds).

  For more information about the example and GPS library, check the github page README at https://github.com/Velleman/VMA430_GPS_Module
  For more informarion about the GPS module, consult the manual at the WPI430 product page on whadda.com
  
 */

#include <VMA430_GPS.h>     // Include the GPS module library
#include <SoftwareSerial.h> // Include the software serial library

SoftwareSerial ss(3, 2); // RX, TX
VMA430_GPS gps(&ss);     // Pass the softwareserial connection info the the GPS module library

void setup()
{
  Serial.begin(9600);
  Serial.println("hello");
  gps.begin(9600); // Sets up the GPS module to communicate with the Arduino over serial at 9600 baud
  gps.setUBXNav(); // Enable the UBX mavigation messages to be sent from the GPS module
}

void loop()
{
  if (gps.getUBX_packet()) // If a valid GPS UBX data packet is received...
  {
    gps.parse_ubx_data(); // Parse the new data
    if (gps.utc_time.valid) // If the utc_time passed from the GPS is valid...
    {
      // Print UTC time hh:mm:ss
      Serial.println();
      Serial.print("UTC time: ");
      Serial.print(gps.utc_time.hour);
      Serial.print(":");
      Serial.print(gps.utc_time.minute);
      Serial.print(":");
      Serial.print(gps.utc_time.second);
      Serial.println();
    }
    // Print location (latitude/longitude)
    Serial.println();
    Serial.print("location: ");
    Serial.print("Lat: ");
    Serial.print(gps.location.latitude, 8); // to 8 decimals
    Serial.print(" Long: ");
    Serial.print(gps.location.longitude, 8); // to 8 decimals
  }

  delay(10);
}
