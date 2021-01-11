#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <BigNumbers_I2C.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <OneWire.h> 
#include <DallasTemperature.h>
#include <string.h>
#include <max7219.h>
#define LEFT 0
#define RIGHT 1
#define TIMEZONE 3
#define ONE_WIRE_BUS 2 

// GYRO
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
int lcd_loop_counter;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output = 0, angle_roll_output = 0, max_pitch = 0, max_roll = 0;


//other
OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature sensors(&oneWire);
TinyGPS gps;
SoftwareSerial ss(4, 5);

//DISPLAY
LiquidCrystal_I2C lcd(0x27,16,2);
BigNumbers_I2C bigNum(&lcd);

//BUTTONS
const byte interruptPin = 3;
volatile byte state = LOW;
int debouncing_time = 300;
volatile unsigned long last_millis;
bool isFirst = true;

MAX7219 max7219;

volatile byte view = 0;
String speed = "-";
float topSpeed = 0;
String time;
String temp;
int dateHour = 0;
int dateMin = 0;
bool isConnected = false;
int satellites = 0;

void setup() {
  Wire.begin();                                                        //Start I2C as master
  ss.begin(9600);
  Serial.begin(9600);
  Serial.print("start");
  max7219.Begin();
  max7219.DisplayText("GS 500 F",LEFT);
  sensors.begin(); 
  lcd.begin();  
  bigNum.begin();
  lcd.backlight();
  lcd.setCursor(0,0);
  setup_mpu_6050_registers();
  lcd.setCursor(0,0);                                                  //Set the LCD cursor to position to position 0,0
  lcd.print("Calibrating");                                    //Print text to screen
  lcd.setCursor(0,1);                                                  //Set the LCD cursor to position to position 0,1
  for (int cal_int = 0; cal_int < 1600 ; cal_int ++){                  //Run this code 2000 times
    if(cal_int % 100 == 0){
      lcd.print(".");                              //Print a dot on the LCD every 125 readings
    }
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  gyro_x_cal /= 1600;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 1600;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 1600;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), debounceInterrupt, CHANGE);
}

void loop() {
  unsigned long chars;
  unsigned short sentences, failed;
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (ss.available())
    {
      char c = ss.read();
      if (gps.encode(c))
        print_speed(gps.f_speed_kmph(), TinyGPS::GPS_INVALID_F_SPEED, 6, 2);
        print_date();
        if(gps.satellites() != TinyGPS::GPS_INVALID_SATELLITES) satellites = gps.satellites();
        else satellites = 0;
    }
  }
  read_mpu_6050_data(); 
  sensors.requestTemperatures();
  temp = String(sensors.getTempCByIndex(0));
  temp = temp.substring(0,2) + " C";
  float tmp;
  char topSpeedText[16];
  char curAngleText[16];
  char maxAngleText[16];
  switch (view) {
    case 0:{
      lcd.clear();
      if(satellites == 0) lcd.print("Connecting...");
      else {
              bigNum.displayLargeInt(dateHour, 0, 0, 2, true);
              bigNum.displayLargeInt(dateMin, 8, 0, 2, true);
              lcd.setCursor(15,1);
              lcd.print(String(satellites));
      }
      break;
    }
      
    case 1: {
      dtostrf(topSpeed,3,0,topSpeedText);
      lcd.clear();
      lcd.print("TopSpeed:" + String(topSpeedText) + "km/h");
      lcd.setCursor(0,1);          
      lcd.print("RideTime:  " + String(millis()/1000/60) + "min");
      break;
    }
    
    case 2: {
      dtostrf(max_roll,3,0,maxAngleText);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Max Lean:" + String(maxAngleText));
      lcd.setCursor(0,1);
      dtostrf(max_pitch,3,0,maxAngleText);
      lcd.print("Max Pitch:" + String(maxAngleText));
      break;
    }
    
    case 3: {
        tmp = abs(angle_roll_output);
        dtostrf(tmp,3,0,curAngleText);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Cur Lean:" + String(curAngleText));
        lcd.setCursor(0,1);
        tmp = abs(angle_pitch_output);
        dtostrf(tmp,3,0,curAngleText);
        lcd.print("Cur Pitch:" + String(curAngleText));
        break;
    }

  }
  String dispText;
  char charBuf[64];
  max7219.Clear();
  dispText = speed;
  dispText.toCharArray(charBuf, 64);
  max7219.DisplayText(charBuf,LEFT);
  dispText = temp;
  dispText.toCharArray(charBuf,64);
  max7219.DisplayText(charBuf, RIGHT);

}

void debounceInterrupt() {
  if((long)(millis() - last_millis) >= debouncing_time) {
    changeView();
    last_millis = millis();
  }
}

void changeView() {
  if(view == 4) view = 0;
  else view++;
}
 
static char* print_speed(float val, float invalid, int len, int prec)
{
  if (val == invalid)
  {
    
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
    if(val > topSpeed) topSpeed = val;
    dtostrf(val,3,0,fText);
    speed = String(fText);

  }
  smartdelay(0);
}

static void print_date()
{
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if (age == TinyGPS::GPS_INVALID_AGE){
    return;
  }
  else
  {
    dateMin = minute;
    dateHour = hour + TIMEZONE;
    char sz[16];
    sprintf(sz, "%02d%02d",
        hour+TIMEZONE, minute);
    time = String(sz);
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

void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}

void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B); 
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  acc_x = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
  acc_y = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
  acc_z = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  gyro_x = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable

  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value
  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
  
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel
  
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

  if(set_gyro_angles){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{                                                                //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                                            //Set the IMU started flag
  }
  
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
  if (abs(angle_roll_output) > max_roll) max_roll = abs(angle_roll_output);
  if (abs(angle_pitch_output) > max_pitch) max_pitch = abs(angle_pitch_output);
}
