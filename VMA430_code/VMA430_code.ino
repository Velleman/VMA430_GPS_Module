#include <VMA430_GPS.h>
#include <SoftwareSerial.h>

SoftwareSerial ss(2, 3); // RX, TX
VMA430_GPS gps(&ss);
void setup()
{
  Serial.begin(9600);
  Serial.println("hello");
  gps.begin(9600);
}

void loop()
{
  delay(10);
}

