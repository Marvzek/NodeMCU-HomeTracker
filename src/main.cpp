#include <Arduino.h>

const int EarthRadius = 6371000;
const float EssingenLatitudeDeg = 48.7907211;
const float EssingenLongitudeDeg = 9.9525687;

const float EssingenLatitudeRad = EssingenLatitudeDeg * DEG_TO_RAD;
const float EssingenLongitudeRad = EssingenLongitudeDeg * DEG_TO_RAD;

float CurrentPointLatitudeDeg = 48.4241634;
float CurrentPointLongitudeDeg = 10.0316937;

float CurrentPointLatitudeRad = CurrentPointLatitudeDeg * DEG_TO_RAD;
float CurrentPointLongitudeRad = CurrentPointLongitudeDeg * DEG_TO_RAD;

float DeltaPhi = 0.0;
float DeltaLambda = 0.0;

float a, c, Distance;
float y, x, Theta, Bearing;
void setup()
{
  Serial.begin(9600);
  Serial.println("Booting");
}

void loop()
{
  // put your main code here, to run repeatedly:

  DeltaPhi = (CurrentPointLatitudeDeg - EssingenLatitudeDeg) * DEG_TO_RAD;
  DeltaLambda = (CurrentPointLongitudeDeg - EssingenLongitudeDeg) * DEG_TO_RAD;
  // a = sin²(Δφ/2) + cos φ1 ⋅ cos φ2 ⋅ sin²(Δλ/2)
  a = sin(DeltaPhi / 2.0) * sin(DeltaPhi / 2.0) + cos(EssingenLatitudeRad) * cos(CurrentPointLatitudeRad) * sin(DeltaLambda / 2.0) * sin(DeltaLambda / 2.0);
  c = 2 * atan2(sqrt(a), sqrt(1.0 - a));

  Distance = EarthRadius * c; // in metres

  Serial.println((String) "Distance=" + Distance);
  // phi = latitude
  // lambda = longitude

  y = sin(CurrentPointLongitudeRad - EssingenLongitudeRad) * cos(CurrentPointLatitudeRad);
  x = cos(EssingenLatitudeRad) * sin(CurrentPointLatitudeRad) - sin(EssingenLatitudeRad) * cos(CurrentPointLatitudeRad) * cos(CurrentPointLongitudeRad - EssingenLongitudeRad);
  Theta = atan2(y, x);
  Bearing = fmodf((Theta * 180.0 / PI + 360.0),360) ; // in degrees
  Serial.println((String) "Bearing=" + Bearing);
  delay(1000);
}