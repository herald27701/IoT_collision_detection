#include <Arduino.h>

#include "Wire.h" 
#include <LiquidCrystal_I2C.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <string.h>
#include <EEPROM.h>
#include <FirebaseESP32.h>

#define FIREBASE_HOST "https://crashlog-b0846-default-rtdb.asia-southeast1.firebasedatabase.app/" //Firebase Host Link
#define FIREBASE_AUTH "3scqx61wsHGpQX33futE5hZThov6BMK7dv6UwKIq" //Firebase Auth Key

FirebaseData firebaseData;
FirebaseJson json;

const char* ssid = "RedmiNote11"; // Wifi name
const char* password = "12345678" ; // Wifi password
LiquidCrystal_I2C lcd(0x27,20,4);

#define EEPROM_SIZE 8

#define MPU6050_ADDR              0x68 // Alternatively set AD0 to HIGH  --> Address = 0x69
#define MPU6050_ACCEL_CONFIG      0x1C // Accelerometer Configuration Register
#define MPU6050_PWR_MGT_1         0x6B // Power Management 1 Register
#define MPU6050_INT_PIN_CFG       0x37 // Interrupt Pin / Bypass Enable Configuration Register
#define MPU6050_INT_ENABLE        0x38 // Interrupt Enable Register
#define MPU6050_LATCH_INT_EN      0x05 // Latch Enable Bit for Interrupt 
#define MPU6050_ACTL              0x07 // Active-Low Enable Bit
#define MPU6050_WOM_EN            0x06 // Wake on Motion Enable bit
#define MPU6050_WOM_THR           0x1F // Wake on Motion Threshold Register
#define MPU6050_MOT_DUR           0x20 // Motion Detection Duration Register
#define MPU6050_ACCEL_INTEL_CTRL  0x69 // Accelaration Interrupt Control Register
#define MPU6050_SIGNAL_PATH_RESET 0x68 // Signal Path Reset Register
#define MPU6050_CONFIG            0x1A ///< General configuration register

#define SENSITIVITY 55
#define MEASUREMENT_UNCERTAIN 10
#define PROCESS_NOISE 0.1

String UrlThingspeak = "https://api.thingspeak.com/update?api_key=7U2E8F0HX0XRDYDS&field1=0"; //Thinkspeak API key
String httpGETRequest(const char* Url); // predefine function
void displayGps(); // predefine function

volatile int16_t accX, accY, accZ; // Raw register values (acceleration)
volatile int count = 0;
volatile float ax[2] = {0, 0};
volatile float ay[2] = {0, 0};
volatile float az[2] = {0, 0};
volatile float dX, dY, dZ;

volatile float global_lat, global_lng;
volatile float previous_lat, previous_lng;
char global_date[12], global_time[12];

static const int interruptPin=2;
static const int buzzPin=1;
static const int GPS_RXPin = 18, GPS_TXPin = 19;
static const int SIM_RXPin = 4, SIM_TXPin = 5;
static const uint32_t GPSBaud = 9600;
static const uint32_t SimBaud = 115200;

volatile bool gpsEvent = false; //gps flag
volatile bool impactEvent = false; // impact flag
volatile bool accEvent = false; // event flag 
volatile bool ontimerEvent = false; // timer event flag 
volatile uint64_t last_accEvent_time = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

hw_timer_t* timer = NULL;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
HardwareSerial gpsSerial(1); // use UART1
HardwareSerial simSerial(0); // use UART0

void updateSerial()
{
  delay(500);
  while (Serial.available()) 
  {
    simSerial.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while(simSerial.available()) 
  {
    Serial.write(simSerial.read());//Forward what Software Serial received to Serial Port
  }
}

void simInit()
{
  Serial.println("Initializing...");
  delay(1000);

  simSerial.println("AT"); //Once the handshake test is successful, it will back to OK
  updateSerial();
  simSerial.println("AT+CSQ"); //Signal quality test, value range is 0-31 , 31 is the best
  updateSerial();
  simSerial.println("AT+CCID"); //Read SIM information to confirm whether the SIM is plugged
  updateSerial();
  simSerial.println("AT+CREG?"); //Check whether it has registered in the network
  updateSerial();
}

void simSendSMS(float lat, float lon)
{
  char msg[200];
  sprintf(msg, "Vehicle crash at https://maps.google.com/?q=%.6f,%.6f ", lat, lon);

  simSerial.println("AT"); //Once the handshake test is successful, it will back to OK
  updateSerial();
  simSerial.println("AT+CMGF=1"); // Configuring TEXT mode
  updateSerial();
  simSerial.println("AT+CMGS=\"+84707798881\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
  updateSerial();
  simSerial.print(msg); //text content
  updateSerial();
  simSerial.write(26);
}

void firebaseUpdate(char date[], char time[])
{
  char path[50]; 
  char lat_path[100];
  char lng_path[100];
  strcat(path, date);
  strcat(path, time);
  strcat(lat_path, path);
  strcat(lat_path, "/lat");
  strcat(lng_path, path);
  strcat(lng_path, "/long");
  Firebase.setString(firebaseData, lat_path, String(global_lat, 6));
  Firebase.setString(firebaseData, lng_path, String(global_lng, 6));
}

void IRAM_ATTR onTimer()
{
  ontimerEvent = true;
}

void writeRegister(uint16_t reg, uint16_t value){
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

void setInterrupt(byte threshold){
//writeRegister(MPU6050_SIGNAL_PATH_RESET, 0b00000111);  // not(?) needed
//writeRegister(MPU6050_INT_PIN_CFG, 1<<MPU6050_ACTL); // 1<<MPU6050_LATCH_INT_EN
  writeRegister(MPU6050_ACCEL_CONFIG, 0b00000111);
  writeRegister(MPU6050_WOM_THR, threshold); 
  writeRegister(MPU6050_MOT_DUR, 0b00000001);  // set duration (LSB = 1 ms)
//writeRegister(MPU6050_ACCEL_INTEL_CTRL, 0x15);  // not needed (?)
  writeRegister(MPU6050_INT_ENABLE, 1<<MPU6050_WOM_EN);
}

void IRAM_ATTR motion(){
  portENTER_CRITICAL_ISR(&mux);
  accEvent = true;
  //last_accEvent_time = esp_timer_get_time();
  detachInterrupt(2);
  portEXIT_CRITICAL_ISR(&mux);
}

float KalmanX(float U)
{
  static const float R = MEASUREMENT_UNCERTAIN;
  static float Q = PROCESS_NOISE;
  static float P = 0;
  static float U_hat = 0;
  static double K = 0;

  K = P/(P+R);
  U_hat = U_hat + K*(U-U_hat);

  P = (1-K)*P+Q;

  return U_hat;
}
float KalmanY(float U)
{
  static const float R = MEASUREMENT_UNCERTAIN;
  static float Q = PROCESS_NOISE;
  static float P = 0;
  static float U_hat = 0;
  static double K = 0;

  K = P/(P+R);
  U_hat = U_hat + K*(U-U_hat);

  P = (1-K)*P+Q;

  return U_hat;
}
float KalmanZ(float U)
{
  static const float R = MEASUREMENT_UNCERTAIN;
  static float Q = PROCESS_NOISE;
  static float P = 0;
  static float U_hat = 0;
  static double K = 0;

  K = P/(P+R);
  U_hat = U_hat + K*(U-U_hat);

  P = (1-K)*P+Q;

  return U_hat;
}

void Read_RawValue(uint16_t deviceAddress, uint16_t regAddress)
{
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart.
  // As a result, the connection is kept active.
  Wire.requestFrom(deviceAddress, 6, true); // request a total of 7*2=14 registers (start from 0x3B to 0x48 all value of the sensors)
  // In this case we just want to read the accelerometer, so 3*2=6 registers

  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same int16_t variable
  accX = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accY = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accZ = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  // Apply the Kalman Filter
  accX = KalmanX(accX);
  accY = KalmanY(accY);
  accZ = KalmanZ(accZ);
}

void sampling0()
{
  ax[0] = accX; // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  ay[0] = accY; // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  az[0] = accZ; // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  
  // Serial.print("AcX0:"); Serial.print(ax[0]/16384.0);
  // Serial.print(",");
  // Serial.print("AcY0:"); Serial.print(ay[0]/16384.0);
  // Serial.print(",");
  // Serial.print("AcZ0:"); Serial.print(az[0]/16384.0);
  // Serial.println("");
}

void sampling1()
{
  
  ax[1] = accX; // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  ay[1] = accY; // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  az[1] = accZ; // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  
  // Serial.print("AcX1:"); Serial.print(ax[1]/16384.0);
  // Serial.print(",");
  // Serial.print("AcY1:"); Serial.print(ay[1]/16384.0);
  // Serial.print(",");
  // Serial.print("AcZ1:"); Serial.print(az[1]/16384.0);
  // Serial.println("");
}

void impactHandler(float dX, float dY, float dZ)
{
  pinMode(buzzPin, OUTPUT);
  lcd.setCursor(0,0);
  if (dX > 0.03)
  {
    if (dZ > 0.03)
    {
      if (dY >= 0.02)
      {
        if (((global_lat-previous_lat)<0.0001) && ((global_lng-previous_lng)<0.0001))
        {
          lcd.print("Accident");
          last_accEvent_time = esp_timer_get_time();
          impactEvent = true;
          firebaseUpdate(global_date, global_time);
          for(int i = 0; i<3; i++)
          {
            digitalWrite(buzzPin, HIGH);  // turn the LED on (HIGH is the voltage level)
            delay(500);                      // wait for a second
            digitalWrite(buzzPin, LOW);   // turn the LED off by making the voltage LOW
            delay(500);
          }
          simSendSMS(global_lat, global_lng);
        }
        else
        {
          lcd.print("High Bump");
          last_accEvent_time = esp_timer_get_time();
          impactEvent = true;
        }
      }
      else
      {
        lcd.print("High Bump");
        last_accEvent_time = esp_timer_get_time();
        impactEvent = true;
        
      }
    }
    else
    {
      lcd.print("Normal Drive");
    }
  }
  else
  {
    if (dZ > 0.03)
    {
      lcd.print("Low Bump");
      last_accEvent_time = esp_timer_get_time();
      impactEvent = true;
      
    }
    else
    {
      lcd.print("Normal Drive");
    }
  }

  // Uncomment this to upload data to Thinkspeak
  // char para[50];
  // sprintf(para,"&field1=%.2f&field2=%.2f&field3=%.2f",dX,dY,dZ);
  // String Url = UrlThingspeak + String(para);
  // httpGETRequest(Url.c_str());
}
void setup() {
  Wire.begin();
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
  gpsSerial.begin(GPSBaud, SERIAL_8N1, GPS_RXPin, GPS_TXPin); // rx, tx //init gps baud
  simSerial.begin(SimBaud, SERIAL_8N1, SIM_RXPin, SIM_TXPin); // rx, tx //init sim baud
  lcd.init();
  // Turn off backlight for power saving
  //lcd.backlight();

  simInit(); //Init sim module

  while(!Serial);
  {
    Serial.println("Initializing MPU6050...");
    lcd.print("Initializing MPU6050...");
    delay(1000);
    lcd.clear();
  } // Wait for Serial Monitor to open
  
  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() == 0)
  {
    Serial.println("MPU6050 is detected.");
    lcd.print("MPU6050 is detected.");
    writeRegister(MPU6050_PWR_MGT_1, 0x0); // PWR_MGMT_1 register
    writeRegister(MPU6050_CONFIG, 0x0); // enable low pass filter
    writeRegister(MPU6050_ACCEL_CONFIG, 0x0); // scale range of the accelerometer (resolution) //maximum length 16 bit = 32768 raw/ each g
    setInterrupt(SENSITIVITY); // set Wake on Motion Interrupt / Sensitivity; 1(highest sensitivity) - 255
    
    pinMode(interruptPin, INPUT);

    //init timer interupt
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 1000000, true);
    timerAlarmEnable(timer);

    attachInterrupt(digitalPinToInterrupt(interruptPin), motion, RISING);

    delay(1000);
    lcd.clear();
  }
  else
  {
    Serial.println("MPU6050 is not detected.");
    lcd.print("MPU6050 is not detected.");
    while (1); // Loop indefinitely
  }
  
  //Wifi
  WiFi.begin(ssid,password);
  Serial.println("connecting");
  lcd.print("connecting");
  int timeout = 0;
  for (int i = 0; i<20; i++)
  {
    if (timeout == 19)
    {
      lcd.clear();
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      return;
    }
    if (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.print(".");
      lcd.print(".");
    }
    else
    {
      Serial.println("");
      Serial.print("Connected to WiFi network with IP Address: ");
      Serial.println(WiFi.localIP());
      lcd.clear();
      lcd.print("Connected:");
      lcd.setCursor(0, 1);
      lcd.print(WiFi.localIP());
      delay(1000);
      lcd.clear();
      Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
      Firebase.reconnectWiFi(true);
      return;
    }
    timeout++;
  }
}



void loop() {
  //updateSerial();
  // This sketch displays information every time a new sentence is correctly encoded.
  while (gpsSerial.available() > 0)
    if (gps.encode(gpsSerial.read()))
      gpsEvent = true;

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    gpsEvent = false;
    while(true);
  }

  if (gpsEvent)
  {
    displayGps();
  }

  if(accEvent){
    Read_RawValue(MPU6050_ADDR, 0x3B);
    if (count % 2 == 0)
    {
      sampling0();
    }
    else
    {
      sampling1();
    }
    dX = (abs(ax[1]-ax[0])/16384.0);
    dY = (abs(ay[1]-ay[0])/16384.0);
    dZ = (abs(az[1]-az[0])/16384.0);

    lcd.setCursor(16,0);
    lcd.print(dX, 2);
    lcd.setCursor(16,1);
    lcd.print(dY, 2);
    lcd.setCursor(16,2);
    lcd.print(dZ, 2);

    
    // Pulse test
    Serial.print("dX:"); Serial.print(dX);
    Serial.print(",");
    Serial.print("dY:"); Serial.print(dY);
    Serial.print(",");
    Serial.print("dZ:"); Serial.print(dZ);
    Serial.println("");

    impactHandler(dX, dY, dZ);

    count++;
    accEvent = false;
    attachInterrupt(digitalPinToInterrupt(interruptPin), motion, RISING);
    ontimerEvent = true;
  }
  

  if (ontimerEvent) {
    ontimerEvent = false;
    if ( (esp_timer_get_time() - last_accEvent_time >= 5000000) )
    {
      dX = 0.0;
      dY = 0.0;
      dZ = 0.0;

      lcd.setCursor(13,0);
      lcd.print("dX:");
      lcd.setCursor(13,1);
      lcd.print("dY:");
      lcd.setCursor(13,2);
      lcd.print("dZ:");

      lcd.setCursor(16,0);
      lcd.print(dX, 2);
      lcd.setCursor(16,1);
      lcd.print(dY, 2);
      lcd.setCursor(16,2);
      lcd.print(dZ, 2);

      if (impactEvent)
      {
        lcd.clear();
        impactEvent = false;
      }
    }
    
  }
}

String httpGETRequest(const char* Url)
{
  HTTPClient http;
  http.begin(Url);
  int responseCode = http.GET();
  String responseBody = "{}";
  if(responseCode > 0)
  {
    Serial.print("responseCode:");
    Serial.println(responseCode);
    responseBody = http.getString();
  }
  else
  {
    Serial.print("Error Code: ");
    Serial.println(responseCode);
  }
  http.end();
  return responseBody;
}

void displayGps()
{
  if (gps.location.isValid())
  {
    previous_lat = EEPROM.readFloat(0);
    previous_lng = EEPROM.readFloat(4);
    lcd.setCursor(0,1);
    lcd.print(gps.location.lat(), 6);
    lcd.setCursor(0,2);
    lcd.print(gps.location.lng(), 6);
    global_lat = gps.location.lat();
    global_lng = gps.location.lng();
    EEPROM.writeFloat(0, global_lat);
    EEPROM.commit();
    EEPROM.writeFloat(4, global_lng);
    EEPROM.commit();
  }
  else
  {
    previous_lat = EEPROM.readFloat(0);
    previous_lng = EEPROM.readFloat(4);
    global_lat = EEPROM.readFloat(0);
    global_lng = EEPROM.readFloat(4);
    lcd.setCursor(0,1);
    lcd.print(global_lat, 6);
    lcd.setCursor(0,2);
    lcd.print(global_lng, 6);
  }

  if (gps.date.isValid())
  {
    lcd.setCursor(11,3);
    lcd.print(gps.date.day());
    lcd.print(F("/"));
    lcd.print(gps.date.month());
    lcd.print(F("/"));
    lcd.print(gps.date.year());
    sprintf(global_date, "/%d|%d|%d", gps.date.day(), gps.date.month(), gps.date.year());
  }
  if (gps.time.isValid())
  {
    lcd.setCursor(0,3);
    lcd.print(gps.time.hour()+7);
    lcd.print(F(":"));
    lcd.print(gps.time.minute());
    lcd.print(F(":"));
    lcd.print(gps.time.second());
    sprintf(global_time, "/%d:%d:%d", (gps.time.hour()+7), gps.time.minute(), gps.time.second());
  }
}