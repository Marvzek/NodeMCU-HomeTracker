#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include "SSD1306Wire.h"
#include <DFRobot_QMC5883.h>
#include "images.h"

// REAL DISTANCE
const float EssingenLatitudeDeg = 48.80833333;
const float EssingenLongitudeDeg = 10.0225;


// TEST Greatest Distance
// const float EssingenLatitudeDeg = -48.7907211;
// const float EssingenLongitudeDeg = -169.9525687;

// TEST within 10 km
//const float EssingenLatitudeDeg = 48.4039701;
//const float EssingenLongitudeDeg = 9.9271;

TinyGPSPlus gps;
SoftwareSerial SerialGPS(D7, D6); // RX, TX

// //Scanning...
// //I2C device found at address 0x1E  !
// //I2C device found at address 0x3C  !

SSD1306Wire display(0x3C, D2, D1); // ADDRESS, SDA, SCL
DFRobot_QMC5883 compass(&Wire, 0x1E);
double Distance, CourseTo;
int Satellites = 0;
void setup()
{
  // Serial.begin(9600);
  // Serial.println("Booting");

  SerialGPS.begin(9600);

  Wire.begin(D2, D1);
  display.init();
  display.clear();
  // display.invertDisplay();
  //display.flipScreenVertically();
  display.setFont(ArialMT_Plain_24);
  display.drawString(0, 26, "Hello world");
  display.display();
  int i = 0;
  while (!compass.begin())
  {
    // Serial.println("Could not find a valid 5883 sensor, check wiring!");
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 16, (String) "No Compass " + i);
    i++;
    display.display();
    delay(500);
  }
}
void loop()
{

  while (SerialGPS.available() > 0)
    gps.encode(SerialGPS.read());

  if (gps.location.isUpdated())
  {
    // Serial.print("LAT=");
    // Serial.print(gps.location.lat(), 6);
    // Serial.print("LNG=");
    // Serial.print(gps.location.lng(), 6);
    // Serial.print("Sat=");
    // Serial.print(gps.satellites.value());
    // Serial.print("Distance=");
    Distance = gps.distanceBetween(gps.location.lat(), gps.location.lng(), EssingenLatitudeDeg, EssingenLongitudeDeg);
    CourseTo = gps.courseTo(gps.location.lat(), gps.location.lng(), EssingenLatitudeDeg, EssingenLongitudeDeg);
    
    Satellites = gps.satellites.value();
    // Serial.print("Entfernung nach Essingen: ");
    // Serial.print(Distance);
    // Serial.print("Kurs: ");
    // Serial.println(CourseTo);

    // readGPS();
    // delay(1000);
  }
  /**
   * @brief  Set declination angle on your location and fix heading
   * @n      You can find your declination on: http://magnetic-declination.com/
   * @n      (+) Positive or (-) for negative
   * @n      For Bytom / Poland declination angle is 4'26E (positive)
   * @n      Formula: (deg + (min / 60.0)) / (180 / PI);
   */
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / PI);
  compass.setDeclinationAngle(declinationAngle);
  sVector_t mag = compass.readRaw();
  compass.getHeadingDegrees();
  // Serial.print("X:");
  // Serial.print(mag.XAxis);
  // Serial.print(" Y:");
  // Serial.print(mag.YAxis);
  // Serial.print(" Z:");
  // Serial.println(mag.ZAxis);
  // Serial.print("Degress = ");
  // Serial.println(mag.HeadingDegress);
  // delay(100);
  display.setFont(ArialMT_Plain_16);
  display.clear();

  if (Satellites < 1)
  {
    if ((millis() / 500) % 2)
    {
      display.drawXbm(102, 0, Satellite_width, Satellite_height, Satellite_bits);
    }
  }
  else
  {
    display.drawXbm(102, 0, 16, 16, Satellite_bits);
  }

  display.drawString(120, 0, (String)Satellites);

  int distanceX = 128;
  int distanceY = 20;
  int distanceUnitX = 128;
  int distanceUnitY = 46;

  int tempDistance = 0;

  if (Distance < 9999.9)
  {
    tempDistance = Distance;
    display.setFont(ArialMT_Plain_24);
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(distanceX, distanceY, (String)tempDistance);
    display.setFont(ArialMT_Plain_16);
    display.drawString(distanceUnitX, distanceUnitY, "[m]");
    display.setTextAlignment(TEXT_ALIGN_LEFT);
  }

  else if (Distance > 9999.9)
  {
    tempDistance = Distance / 1000;
    display.setFont(ArialMT_Plain_24);
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(distanceX, distanceY, (String)tempDistance);
    display.setFont(ArialMT_Plain_16);
    display.drawString(distanceUnitX, distanceUnitY, "[km]");
    display.setTextAlignment(TEXT_ALIGN_LEFT);
  }

  int circleX = 36;
  int circleY = 32;
  int circleRadius = 31;
  int smallCircleRadiusHi = 20;
  int smallCircleRadiusLo = 5;

  display.drawCircle(circleX, circleY, circleRadius);
  display.drawLine(circleX, 0, circleX, 6);
  display.drawLine(circleX +1, 0, circleX+1, 6);
  display.drawLine(circleX -1, 0, circleX-1, 6);
  if (Satellites < 1)
  {

    display.fillCircle(circleX, circleY, smallCircleRadiusLo + ((((millis() + 500) % 1500) / 1500.0) * (smallCircleRadiusHi - smallCircleRadiusLo)));
  }
  else
  {

    double headingDiffMargin = 5.0;
    double headingDiff = mag.HeadingDegress - CourseTo;
    headingDiff += 180;
    headingDiff = fmod(headingDiff,360);
    
    if (abs(headingDiff) <= headingDiffMargin)
    {
      display.invertDisplay();
    }
    else
    {
      display.normalDisplay();
    }
    display.setFont(ArialMT_Plain_10);

    int angleOffset = 120;
    int radius1 = 25;
    int radius2 = 15;
    double tempHeadingDiff = headingDiff+180.0;
    display.fillTriangle(
        circleX + radius1 * cos(tempHeadingDiff * DEG_TO_RAD),
        circleY + radius1 * sin(tempHeadingDiff * DEG_TO_RAD),
        circleX + radius2 * cos((tempHeadingDiff - angleOffset) * DEG_TO_RAD),
        circleY + radius2 * sin((tempHeadingDiff - angleOffset) * DEG_TO_RAD),
        circleX + radius2 * cos((tempHeadingDiff + angleOffset) * DEG_TO_RAD),
        circleY + radius2 * sin((tempHeadingDiff + angleOffset) * DEG_TO_RAD));

    display.drawLine(circleX, circleY, circleX + radius1 * cos(tempHeadingDiff * DEG_TO_RAD), circleY + radius1 * sin(tempHeadingDiff * DEG_TO_RAD));
    display.drawLine(circleX, circleY, circleX + radius2 * cos((tempHeadingDiff - angleOffset) * DEG_TO_RAD), circleY + radius2 * sin((tempHeadingDiff - angleOffset) * DEG_TO_RAD));
    display.drawLine(circleX, circleY, circleX + radius2 * cos((tempHeadingDiff + angleOffset) * DEG_TO_RAD), circleY + radius2 * sin((tempHeadingDiff + angleOffset) * DEG_TO_RAD));

    //display.drawString(70, 54, (String)headingDiff);
  }

   //display.drawString(0, 40, (String)mag.HeadingDegress);

  //display.drawString(70, 0, (String)CourseTo);
  
  display.display();

  // delay(100);
}
