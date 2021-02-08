#include <VMA430_GPS.h>
#include <SoftwareSerial.h>

SoftwareSerial ss(2, 3); // RX, TX
VMA430_GPS gps(&ss);
void setup()
{
  Serial.begin(9600);
  Serial.println("hello");
  gps.begin(9600);
  gps.setUBXNav();
  gps.getUBX_packet();
}

void loop()
{
  //gps.getconfig();
  if(gps.getUBX_packet())
  {
    gps.parse_ubx_data();
    if(gps.utc_time.valid)
    {
      Serial.println();
      Serial.print("UTC time: ");
      Serial.print(gps.utc_time.hour);
      Serial.print(":");
      Serial.print(gps.utc_time.minute);
      Serial.print(":");
      Serial.print(gps.utc_time.second);
      Serial.println();
    }
     Serial.println();
    Serial.print("location: ");
    Serial.print("Lat: ");
    Serial.print(gps.location.latitude, 8);
     Serial.print(" Long: ");
     Serial.print(gps.location.longitude, 8);
  }
  
   
  delay(10);
}

