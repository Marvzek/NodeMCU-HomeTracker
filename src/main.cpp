#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Wire.h>        // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306Wire.h" // legacy include: `#include "SSD1306.h"`

const float EssingenLatitudeDeg = 48.7907211;
const float EssingenLongitudeDeg = 9.9525687;

TinyGPSPlus gps;
SoftwareSerial SerialGPS(D7, D6); // RX, TX

SSD1306Wire display(0x3c, D5, D4); // ADDRESS, SDA, SCL

void setup()
{
  Serial.begin(9600);
  Serial.println("Booting");

  SerialGPS.begin(9600);

  display.init();
  display.clear();
  // display.invertDisplay();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_24);
  display.drawString(0, 26, "Hello world");
  display.display();
}
void loop()
{

  while (SerialGPS.available() > 0)
    gps.encode(SerialGPS.read());

  if (gps.location.isUpdated())
  {
    Serial.print("LAT=");
    Serial.print(gps.location.lat(), 6);
    Serial.print("LNG=");
    Serial.print(gps.location.lng(), 6);
    Serial.print("Sat=");
    Serial.print(gps.satellites.value());
    Serial.print("Distance=");
    double distanceKm =
        gps.distanceBetween(gps.location.lat(), gps.location.lng(), EssingenLatitudeDeg, EssingenLongitudeDeg) / 1000.0;
    double courseTo =
        gps.courseTo(gps.location.lat(), gps.location.lng(), EssingenLatitudeDeg, EssingenLongitudeDeg);
    Serial.print("Entfernung nach Essingen: ");
    Serial.print(distanceKm);
    Serial.print("Kurs: ");
    Serial.println(courseTo);

    //display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
    display.setFont(ArialMT_Plain_10);

    display.clear();
    display.drawString(0, 20, (String)distanceKm);
    display.display();

    // readGPS();
    // delay(1000);
  }
}
/*
float calcDistance()
{
  // put your main code here, to run repeatedly:

  DeltaPhi = (CurrentPointLatitudeDeg - EssingenLatitudeDeg) * DEG_TO_RAD;
  DeltaLambda = (CurrentPointLongitudeDeg - EssingenLongitudeDeg) * DEG_TO_RAD;
  // a = sin²(Δφ/2) + cos φ1 ⋅ cos φ2 ⋅ sin²(Δλ/2)
  a = sin(DeltaPhi / 2.0) * sin(DeltaPhi / 2.0) + cos(EssingenLatitudeRad) * cos(CurrentPointLatitudeRad) * sin(DeltaLambda / 2.0) * sin(DeltaLambda / 2.0);
  c = 2 * atan2(sqrt(a), sqrt(1.0 - a));

  Distance = EarthRadius * c; // in metres

  Serial.println((String) "Distance=" + Distance);
  return 0.0;
}

float calcBearing()
{
  // 	θ = atan2( sin Δλ ⋅ cos φ2 , cos φ1 ⋅ sin φ2 − sin φ1 ⋅ cos φ2 ⋅ cos Δλ )
  // phi = latitude
  // lambda = longitude

  y = sin(CurrentPointLongitudeRad - EssingenLongitudeRad) * cos(CurrentPointLatitudeRad);
  x = cos(EssingenLatitudeRad) * sin(CurrentPointLatitudeRad) - sin(EssingenLatitudeRad) * cos(CurrentPointLatitudeRad) * cos(CurrentPointLongitudeRad - EssingenLongitudeRad);
  Theta = atan2(y, x);
  Bearing = fmodf((Theta * 180.0 / PI + 360.0), 360); // in degrees
  Serial.println((String) "Bearing=" + Bearing);
  return 0.0;
}

int readGPS()
{
  //$GPGGA,214839.00,4826.24617,N,01003.12259,E,1,06,3.11,438.3,M,47.6,M,,*54
  //$

  char msg[300];
  bool continueReading = true;

  int writePosition = 0;

  for (int i = 0; i < 1000; i++)
  {
    Serial.write(SerialGPS.read());
  }
  return 1;
}
*/