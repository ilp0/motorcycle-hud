#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <max7219.h>
#define LEFT 0
#define RIGHT 1

TinyGPS gps;
SoftwareSerial ss(4, 3);

MAX7219 max7219;

float speed = 0;

void setup() {
  ss.begin(9600);
  Serial.begin(115200);
  max7219.Begin();
}

void loop() {
  unsigned long chars;
  unsigned short sentences, failed;
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (ss.available())
    {
      char c = ss.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        print_float(gps.f_speed_kmph(), TinyGPS::GPS_INVALID_F_SPEED, 6, 2);
    }
  }
}
 
static void print_float(float val, float invalid, int len, int prec)
{
  if (val == invalid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
    char fText[16];
    dtostrf(val,4,1,fText);
    max7219.Clear();
    max7219.DisplayText(fText, LEFT);

  }
  smartdelay(0);
}

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}
