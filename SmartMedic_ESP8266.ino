/*************************************************************
  Download latest Blynk library here:
    https://github.com/blynkkk/blynk-library/releases/latest

  Blynk is a platform with iOS and Android apps to control
  Arduino, Raspberry Pi and the likes over the Internet.
  You can easily build graphic interfaces for all your
  projects by simply dragging and dropping widgets.

    Downloads, docs, tutorials: http://www.blynk.cc
    Sketch generator:           http://examples.blynk.cc
    Blynk community:            http://community.blynk.cc
    Follow us:                  http://www.fb.com/blynkapp
                                http://twitter.com/blynk_app

  Blynk library is licensed under MIT license
  This example code is in public domain.

 *************************************************************
  This example runs directly on ESP8266 chip.

  Note: This requires ESP8266 support package:
    https://github.com/esp8266/Arduino

  Please be sure to select the right ESP8266 module
  in the Tools -> Board menu!

  Change WiFi ssid, pass, and Blynk auth token to run :)
  Feel free to apply it to any other example. It's simple!
 *************************************************************/

#define BLYNK_TEMPLATE_ID "TMPLB8vzSAGz"
#define BLYNK_DEVICE_NAME "SmartMedicDevice"

#define BLYNK_PRINT Serial

//Includes:-------------------------------------------------
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include "DHTesp.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <Wire.h>
#include <BH1750.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <OneWire.h>
#include <DallasTemperature.h>

//Defines Blynk-------------------------------------------------

char auth[] = "TOKEN";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "LOGIN";
char pass[] = "PASSWORD";

//Defines-------------------------------------------------
#define ONE_WIRE_BUS 2
#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2
#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH  16
#define MAX_BRIGHTNESS 255

//Library to objects---------------------------------------
BH1750 lightMeter;
DHTesp dht;
MAX30105 particleSensor;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

//Variables---------------------------------------
int adc_light;

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

int IR30102Value;

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

// If using an ESP8266, use this option. Comment out the other options.
// ESP8266 Hardware SPI (faster, but must use certain hardware pins):
//-------------------------------------------------------------------
// pin 14 - SCK is LCD serial clock (SCLK) - this is  on Huzzah ESP8266
// pin 13 - DIN is LCD on an Huzzah ESP8266
// pin 12 - D/C is Data/Command select on an Huzzah ESP8266
// pin 5 -  CS  is LCD chip select  (15)
// pin 4 -  RST is LCD reset  (16)
//-------------------------------------------------------------------
//Adafruit_PCD8544 display = Adafruit_PCD8544(12, 5, 4);
Adafruit_PCD8544 display = Adafruit_PCD8544(14, 13, 12, 15, 16);

static const unsigned char PROGMEM logo16_glcd_bmp[] =
{ B00000000, B11000000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11100000,
  B11110011, B11100000,
  B11111110, B11111000,
  B01111110, B11111111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001101, B01110000,
  B00011011, B10100000,
  B00111111, B11100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000 };

void setup()
{
  Serial.begin(9600);
  Blynk.begin(auth, ssid, pass);  
  
  //Nokia5110-----------------------------------------------
  display.begin();
  display.setContrast(50);// you can change the contrast around to adapt the display
  display.display(); // show splashscreen
  delay(1000);
  display.clearDisplay();   // clears the screen and buffer
  
  //DHT11-----------------------------------------------
  dht.setup(0, DHTesp::DHT11);
  
  //BH1750-----------------------------------------------
  // Initialize the I2C bus (BH1750 library doesn't do this automatically)
  // On esp8266 you can select SCL and SDA pins using Wire.begin(D4, D3);
  Wire.begin();
  lightMeter.begin();

  //MAX30102-----------------------------------------------
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
  
  //18B20------------------------------------------------------------------
   sensors.begin();
  //sensor.setResolution(12);
  
}

void loop(){
  
  //  for (int i=0; i<200; i=i+1){
  //    adc_light = analogRead(A0);
  //    Blynk.virtualWrite(V0, adc_light);
  //  }
  
  //Read DHT11-----------------------------------------------
  float humidity = dht.getHumidity();//V0
  float temperature = dht.getTemperature();//V1
  
  Serial.println("------------------------");
  //Serial.print("light=");
  //Serial.println(adc_light);
  
  Serial.print("humidity=");
  Serial.println(humidity);
  
  Serial.print("temperature=");
  Serial.println(temperature);
  
  //DHT11 to LCD -------------------------------------------------
  display.clearDisplay();   // clears the screen and buffer
  
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setCursor(0,0);
  display.print("T=     H=");
  
  display.setCursor(12,0);
  display.print(temperature);
  display.setCursor(54,0);
  display.print(humidity);
  
  //Read MAX30102 -------------------------------------------------
  float lux = lightMeter.readLightLevel();//V2
  Serial.print("Light= ");
  Serial.print(lux);
  Serial.println(" lx");

  display.setCursor(0,8);
  display.print("Light=");
  display.setCursor(36,8);
  display.print(lux);
  
  //Merge 30102 Bpm -------------------------------------------------
  for (int i = 0; i<200; i++)
  {
        long irValue = particleSensor.getIR();
    if (checkForBeat(irValue) == true)
    {
      //We sensed a beat!
      long delta = millis() - lastBeat;
      lastBeat = millis();
  
      beatsPerMinute = 60 / (delta / 1000.0);
  
      if (beatsPerMinute < 255 && beatsPerMinute > 20)
      {
        rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
        rateSpot %= RATE_SIZE; //Wrap variable
  
        //Take average of readings
        beatAvg = 0;
        for (byte x = 0 ; x < RATE_SIZE ; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
      IR30102Value = irValue;
    }
  }
  
  //MAX30102 to LCD -------------------------------------------------

  Serial.print("Avg BPM=");
  Serial.println(beatsPerMinute);//V3
  
  display.setCursor(0,22);
  display.print("Bpm=");
  display.setCursor(24,22);
  //display.print(beatsPerMinute); 
  
  if (IR30102Value < 70000){
    display.print("No finger?");
  }else{
    display.print(beatsPerMinute);  
  }
  
  //18B20 to LCD -------------------------------------------------  
  float temperature_1820;
  sensors.requestTemperatures();// Send the command to get temperatures
  temperature_1820 = sensors.getTempCByIndex(0);

  Serial.print("Temperature_body=");
  Serial.println(temperature_1820);
  
  display.setCursor(0,30);
  display.print("Tmp=");
  display.setCursor(24,30);
  display.print(temperature_1820);
  
  display.display();
  
  //TX to Blynk-------------------------------------------------
  Blynk.virtualWrite(V0,humidity);
  Blynk.virtualWrite(V1,temperature);
  Blynk.virtualWrite(V2,lux);
  Blynk.virtualWrite(V3,beatsPerMinute);
  Blynk.virtualWrite(V4,temperature_1820);
  
  Blynk.run();
}
