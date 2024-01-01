//
// This is to control a small greenhousse/coldframe
// Automatically water the plants when moisture level drops below setpoint

#include <OneWire.h>
#include <DallasTemperature.h>

#include <Wire.h>
#include <SPI.h>
#include <ArduinoLowPower.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include "Adafruit_TSL2591.h"
#include <ArduinoJson.h>

//
//  The secrets file has two entries
//      #define WiFi_SSID xxxx
//      #define WiFi_Passwd xxxx
//
// You will need to provide this file
//
#include "secrets.h"

Adafruit_BMP3XX bmp ;
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591) ; // pass in a number for the sensor identifier (for your use later)

#include "tsl2951.h"
struct tslDataStruct tslData ;

#include <WiFiNINA.h>
#include <utility/wifi_drv.h>
#include <ArduinoMqttClient.h>

char ssid[] = WiFi_SSID ;              //  your network SSID (name) between the " "
char pass[] = WiFi_Passwd;      // your network password between the " "
int keyIndex = 0;                  // your network key Index number (needed only for WEP)
//int status = WL_IDLE_STATUS;       //connection status

WiFiClient wifiClient ;
MqttClient mqttClient(wifiClient);

//#include <LiquidCrystal_I2C.h>
//LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 column and 2 rows


const char Broker_Host[] = "sds-orange-pi5" ;
const int Broker_Port = 1883 ;
const char Broker_Topic_ENV[] = "/raleigh/home/greenhouse-R02/ENV" ;
const char Broker_Topic_PWR[] = "/raleigh/home/greenhouse-R02/PWR" ;

const int NUMBER_OF_RELAYS = 2 ;
const unsigned long ONE_SECOND      =     1000 ;
const unsigned long TEN_SECONDS     =    10000 ;
const unsigned long FIFTEEN_SECONDS =    15000 ;
const unsigned long ONE_MINUTE      =    60000 ;
const unsigned long FIVE_MINUTES    =   300000 ;
const unsigned long TEN_MINUTES     =   600000 ;
const unsigned long FIFTEEN_MINUTES =   900000 ;
const unsigned long TWENTY_MINUTES  =  1200000 ;

//
// we are using 10bit adc resolution determine voltage per increment of the adc
// Also we need to PRETEND the readings are 0-5VDC of the Current sensor
// even though the MKR WiFi can only measure up to 3.3Vdc the resistor network 
// shifts 5vdc to 3.3vdc so this is OK.
//
const float voltsPerIncr = 5.0/4096.0 ;  

const int HIGH_MOISTURE = 60 ;
const int LOW_MOISTURE = 25 ;

//const int voltageSensor = A4;
//const int currentSensor = A6;
//
// The RP2040 Connect does not like assigning A4 and A6 to const int so...
//

#define voltageSensor A4
#define solarCurrent A6
#define batteryCurrent A5

unsigned long msSincePwrOn ;

// Pin to put interval to 10 seconds
int intervalCheck = 7 ;

// The Digital Pin definitions
int fan1StatePin = 2;
int fan1ControlPin = 3;

int fan2StatePin = 4;
int fan2ControlPin = 5;

int tempSensorsDataPin = 8;  // DB18B20 one-wire-bus

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

// The Analog Pin definitions for ground moisture sensors

struct MoistureSenfor_Info {
  int  sensorPin ;
  long sensorMoisture ;
  long sensorMap[4] ;
} ;

struct MoistureSenfor_Info moistureSensor[3] =
    {{A0, -1, {3370, 2860, 0, 100}},
     {A1, -1, {3370, 2860, 0, 100}},
     {A2, -1, {3370, 2860, 0, 100}}} ;

// Greenhouse datapoints
float greenhouseTemp;
float ambientTemp;
float tmEastTemp ;
float tmWestTemp ;

float lux;

float batVolt;

OneWire oneWire(tempSensorsDataPin);      // setup a oneWire instance
DallasTemperature tempSensors(&oneWire);  // pass oneWire to DallasTemperature library

struct temp_sensor_struct {
  char id[32];
  float temp;
  uint8_t macId[8];
};

struct temp_sensor_struct tempSensorAddrs[4] = { { "Greenhouse", 0.0, { 0x28, 0xCA, 0x99, 0x58, 0xD4, 0xE1, 0x3C, 0xE5 } },         // no tape
                                                 { "Ambient", 0.0, { 0x28, 0x96, 0x53, 0x58, 0xD4, 0xE1, 0x3C, 0x29 } },            // Blue tape
                                                 { "East Thermal Mass", 0.0, {0X28, 0XF2, 0X84, 0X58, 0XD4, 0XE1, 0X3C, 0X39 } },   // Yellow tape
                                                 { "West Thermal Mass", 0.0, {0X28, 0X56, 0XF1, 0X58, 0XD4, 0xE1, 0X3C, 0X06 } } } ;// Red tape

int numTempSensors = sizeof(tempSensorAddrs) / sizeof(tempSensorAddrs[0]) ;

struct fan_struct {
  int statePin;
  int controlPin;
  bool state;
  long transitionDelay ;
};

struct fan_struct relays[2] = { { fan1StatePin, fan1ControlPin, false, 0l},
                                  { fan2StatePin, fan2ControlPin, false, 0l} };


struct {
  float     mbTemp ;
  float     ghTemp ;
  float     ambTemp ;
  float     tmEastTemp ;
  float     tmWestTemp ;
  int       moisture[3] ;
  int       relayState[2] ;
  int       mmhg ;
  uint16_t  ir ;
  uint16_t  lux ;
  float     batVDC ;
  float     sysAmps;
  long      sysAmpsRaw ;
  float     solarAmps ;
  long      solarAmpsRaw ;
} kpis ;

char buffer[1024] ;
DynamicJsonDocument ghENV(1024) ;
DynamicJsonDocument ghPWR(1024) ;


void setRelay(int relay, int desiredState) {
  int relayState ;

  if ( relays[relay].transitionDelay <= 0 ) {
    relayState = digitalRead(relays[relay].statePin) ;
    if (relayState != desiredState) {
     // Togggle the clock on the 74HC74 to change the state
      digitalWrite(relays[relay].controlPin, LOW) ;     // Make sure it is LOW because state changes on LOW-HIGH transition
      delayMicroseconds(100) ;
      digitalWrite(relays[relay].controlPin, HIGH) ;
      delayMicroseconds(100);
      digitalWrite(relays[relay].controlPin, LOW) ;
      relayState = digitalRead(relays[relay].statePin) ;
      if (relayState != desiredState) {
        Serial.print("relay #") ;
        Serial.print(relay) ;
        Serial.print(" - Failed to ");
        if (desiredState = 0) {
          Serial.println("Close") ;
        } else {
          Serial.println("Open") ;
        }
      }
//      relays[relay].transitionDelay = TWENTY_MINUTES ;
      relays[relay].transitionDelay = 3 ;
    }
    relays[relay].state = relayState ;   //Save the surrent fan state
  }
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");

  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}

void enable_WiFi() {
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < "1.0.0") {
    Serial.println("Please upgrade the firmware");
  }
}

void connect_WiFi() {
  // attempt to connect to Wifi network:
int     counter = 0 ;
uint8_t status = WL_IDLE_STATUS;
char    lcdBuffer[32];

  while (true) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);

//    lcd.clear();                 // clear display
//    lcd.setCursor(0, 0);         // move cursor to   (0, 0)
//    lcd.print("WiFi: Connecting") ;
//    lcd.setCursor(0, 1);         // move cursor to   (0, 0)
//    sprintf(lcdBuffer, "Attempt #%d", counter+1);
//    lcd.print(lcdBuffer) ;

    status = WiFi.begin(ssid, pass);
    delay(5000) ;
    Serial.println(status);
    if (status == WL_CONNECTED) {
      break ;
    }

    counter++ ;
    if ((counter%10) == 0) {          // If it takes 10 attempts there is normally some issue and a reset is in order
//      lcd.clear();                 // clear display
//      lcd.setCursor(0, 0);         // move cursor to   (0, 0)
//      lcd.print("WiFi: Failure") ;
//      delay(5000);
      NVIC_SystemReset();
      // Send board to sleep mode
      digitalWrite(NINA_RESETN, HIGH);  // reset nina
      LowPower.deepSleep(10000) ;
      digitalWrite(NINA_RESETN, LOW);  // activate nina
      delay(5000);

    }
  }
//  WiFiDrv::digitalWrite(27, LOW); //Turn Off Blue light
  WiFi.lowPowerMode();

}

unsigned long getInterval() {
  unsigned long result = 0 ;
  if (digitalRead(intervalCheck) == LOW) {
    Serial.println("FIFTEEN_SECONDS") ;
    result = FIFTEEN_SECONDS ;
  } else {
    Serial.println("FIVE_MINUTES") ;
    result = FIVE_MINUTES ;
  }
  return result ;
}

void connectToMqtt() {
  int limit = 10 ;
  char lcdBuffer[20];

  Serial.print("Attempting to connect to the MQTT broker: ");
//  lcd.clear();                 // clear display
//  lcd.setCursor(0, 0);         // move cursor to   (0, 0)
//  lcd.print("Connect to MQTT");        // print message at (0, 0)

  Serial.print(Broker_Host);
  Serial.print(", on port -");
  Serial.println(Broker_Port);
//  WiFiDrv::digitalWrite(25, HIGH); //Turn On Red light
  while (!mqttClient.connect(Broker_Host, Broker_Port)) {
    //Serial.print("MQTT connection failed! Error code = ");
    //Serial.println(mqttClient.connectError());
    limit-- ;
    if (limit <= 0 ) {
//      lcd.clear();
//      lcd.setCursor(0, 0) ;
//      lcd.print("MQTT Failure");
//      lcd.setCursor(0, 1);
//      sprintf(lcdBuffer, "Error Code: %d", mqttClient.connectError()); 
//      lcd.print(lcdBuffer);
      NVIC_SystemReset(); 
      return ;
    }
    delay(1000) ;
  }
//  mqttClient.setKeepAliveInterval(TEN_MINUTES) ;
//  mqttClient.setConnectionTimeout(FIFTEEN_MINUTES) ;
//  WiFiDrv::digitalWrite(25, LOW); //Turn Off Red light
//  lcd.clear();
//  lcd.setCursor(0, 0);         // move cursor to   (0, 0)
//  lcd.print("MQTT: Connected");        // print message at (0, 0)

  return ;
}


void setup() {
  // put your setup code here, to run once:
  msSincePwrOn = 0 ;

  Serial.begin(19200);
  //while (!Serial);

  Serial.println();
  Serial.println("****************************");
  //lcd.init();         // initialize the lcd
  //lcd.backlight();    // Turn on the backlight (Yeah it sucks energy but...)

  pinMode(intervalCheck, INPUT_PULLUP) ;

  //
  // Set up the digital pins for the fan relays
  for (int i = 0; i < 2; i++) {
    pinMode(relays[i].statePin, INPUT) ;
    pinMode(relays[i].controlPin, OUTPUT) ;
    relays[i].transitionDelay = 0 ; // Do not let the relays change for until the second time through the loop
  }

  for (int i = 0; i < 3; i++){
    pinMode(moistureSensor[i].sensorPin, INPUT) ;
  } ;
  
  pinMode(voltageSensor, INPUT) ;      // Voltage Senosr input
  pinMode(solarCurrent, INPUT) ;
  pinMode(batteryCurrent, INPUT) ;      // Current Sensor Input

  analogReadResolution(12);       // only if on an RP2040 or MKR board

//
// Make sure the relays are off.
// Do not let them transition until the second time through 'loop'
//
  setRelay(0, 0) ;
  setRelay(1, 0) ;

// Set up oversampling and filter initialization
  bmp.begin_I2C() ;
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  delay(250) ;
  bmp.performReading() ;  // do a reading now to prevent the first recorded reading from showing a very low set of values
  delay(250) ;
  bmp.performReading() ;  // do a reading now to prevent the first recorded reading from showing a very low set of values

  tsl.begin() ;

  displaySensorDetails() ;    // Display some basic information on this sensor
  configureSensor() ;         // Configure the sensor

  //
  //Setup the temperature sensors
  tempSensors.begin();  // initialize the Dallas Temperature sensor software

  WiFiDrv::pinMode(25, OUTPUT); //GREEN
  WiFiDrv::pinMode(26, OUTPUT); //RED
  WiFiDrv::pinMode(27, OUTPUT); //BLUE

}

void loop() {
  unsigned long delayTime ;
  char outString[80] ;

  msSincePwrOn = millis();
  digitalWrite(LED_BUILTIN, LOW) ;
  digitalWrite(NINA_RESETN, LOW);  // activate nina
  delay(5000);

  enable_WiFi();
  delay(200);               // adding this because there are times that the connect fails and grasping at straws to prevent the board from hanging
  connect_WiFi();

//
// Initialize the data collection for mqtt
//
  kpis.ambTemp = -11.0 ;
  kpis.ghTemp = -11.0 ;
  kpis.mbTemp = -11.0 ;
  kpis.tmEastTemp = -11.0;
  kpis.tmWestTemp = -11.0;
  for (int i=0; i < 3; i++) kpis.moisture[i] = 0 ;
  for (int i=0; i < 2; i++) kpis.relayState[i] = 0 ;
  kpis.ir = 0 ;
  kpis.lux = 0 ;
  kpis.mmhg = 0 ;
  kpis.sysAmps = 0.0 ;
  kpis.batVDC = 0.0 ;

//
// reduce the fan no change interval by one period
//
  for ( int i = 0; i < 2; i++){
    relays[i].transitionDelay = relays[i].transitionDelay - 1 ;
    if (relays[i].transitionDelay < 0) {
      relays[i].transitionDelay = 0;
    }
  }

  // put your main code here, to run repeatedly:
  long currMs ;

  float acsValue ;
  long  acsCum ;
  float acsVoltage ;
  float acsCurrent ;

  // Get the Inside Temperature
  tempSensors.requestTemperatures();

  kpis.ghTemp = tempSensors.getTempC(tempSensorAddrs[0].macId) ;
  kpis.ambTemp = tempSensors.getTempC(tempSensorAddrs[1].macId) ;
  kpis.tmEastTemp = tempSensors.getTempC(tempSensorAddrs[2].macId) ;
  kpis.tmWestTemp = tempSensors.getTempC(tempSensorAddrs[3].macId) ;

  // Get the soil moisture readings

  for (int i = 0; i < 3; i++){
    long moisturePercentage ;
    moistureSensor[i].sensorMoisture = 0 ;
    for (int j=0; j < 50; j++) {
      moistureSensor[i].sensorMoisture += (long)analogRead(moistureSensor[i].sensorPin) ;
      delay(5) ;
    }
    moistureSensor[i].sensorMoisture = moistureSensor[i].sensorMoisture / 50 ;
    kpis.moisture[i] = moistureSensor[i].sensorMoisture ;

//    moisturePercentage = map(moistureSensor[i].sensorMoisture,
//                         moistureSensor[i].sensorMap[0],
//                         moistureSensor[i].sensorMap[1],
//                         moistureSensor[i].sensorMap[2],
//                         moistureSensor[i].sensorMap[3]) ;
//    if ( moisturePercentage < 0 ) {
//      moisturePercentage = 0 ;
//    } ;

//    kpis.moisture[i] = moisturePercentage ;
  
  }

  for (int i = 0; i < NUMBER_OF_RELAYS; i++){
    kpis.relayState[i] = digitalRead(relays[i].statePin) ;
  }

  kpis.batVDC = 0.0 ;
  for (int i=0; i<50; i++){
    kpis.batVDC += ((float)analogRead(voltageSensor) / 4096.0) * 19.0 ;
    delay(5) ;
  }
  kpis.batVDC = kpis.batVDC / 50 ;

  Serial.print("Battery: ");
  Serial.print(kpis.batVDC);
  Serial.println("vDC");
 
  acsCum = 0 ;
  for (int i=0; i < 100; i++) {
    acsCum += analogRead(batteryCurrent) ;
    delay(10);
  }
//
// replace the 0 with the calibration value to get to the midpoint of the adc this could be a positive or negative value
// e.g. if your adc is 12 bits (0-4095) your mid point should be 2048
//      if your adc is 10 bita (0.1023) your midpoint should be 512
// If your test (12 bit adc) shows a reading is 2078 with ZERO current then SUBTRACT 30 to get to the midpoint
// If your test (12 bit adc) shows a reading is 2018 with ZERO current the ADD 30 to the to the midpoint
//
// We are still using 2.5vdc as the midpoint because the resistor network linearly maps 5vdc down to 3.3 for the RP2040 connect or MKR WiFi adc
//
    kpis.sysAmpsRaw = acsCum / 100 ;
    acsValue = kpis.sysAmpsRaw - 17 ;
    acsVoltage = (float)acsValue * voltsPerIncr ;
    acsCurrent = (acsVoltage - 2.5) / 0.185 ;
    acsCurrent = -1 * acsCurrent;               // Because I wired the senors backwards

  kpis.sysAmps = acsCurrent ;

  Serial.print("Battery Cur-Raw: ");
  Serial.println(acsValue);
  Serial.print("ACS Voltage: ");
  Serial.println(acsVoltage,4);
  Serial.print("Battery Cur: ");
  Serial.print(acsCurrent, 4);
  Serial.println("Amps");

  acsCum = 0 ;
  for (int i=0; i < 100; i++) {
    acsCum += analogRead(solarCurrent) ;
    delay(10);
  }
//
// replace the 0 with the calibration value to get to the midpoint of the adc this could be a positive or negative value
// e.g. if your adc is 12 bits (0-4095) your mid point should be 2048
//      if your adc is 10 bita (0.1023) your midpoint should be 512
// If your test (12 bit adc) shows a reading is 2078 with ZERO current then SUBTRACT 30 to get to the midpoint
// If your test (12 bit adc) shows a reading is 2018 with ZERO current the ADD 30 to the to the midpoint
//
// We are still using 2.5vdc as the midpoint because the resistor network linearly maps 5vdc down to 3.3 for the RP2040 connect or MKR WiFi adc
//
    kpis.solarAmpsRaw = acsCum / 100 ;
    acsValue = kpis.solarAmpsRaw + 105 ;
    acsVoltage = (float)acsValue * voltsPerIncr ;
    acsCurrent = (acsVoltage - 2.5) / 0.185 ;
    acsCurrent = -1 * acsCurrent;               // Because I wired the senors backwards

  kpis.solarAmps = acsCurrent ;

  Serial.print("Solar Cur-Raw: ");
  Serial.println(acsValue);
  Serial.print("ACS Voltage: ");
  Serial.println(acsVoltage,4);
  Serial.print("Solar Cur: ");
  Serial.print(acsCurrent, 4);
  Serial.println("Amps");


  if (! bmp.performReading()) {
    kpis.mbTemp = -1.0 ;    // signify a read error
    kpis.mmhg = -1 ;
  } else {
    kpis.mbTemp = bmp.temperature ;
    kpis.mmhg = (int)((bmp.pressure / 100.0) / 1.33322) ;
  }

  advancedRead(&tslData) ;

  kpis.ir = tslData.ir ;
  kpis.lux = tslData.luminosity ;
  if (kpis.lux == 0xFFFF)
    kpis.lux = 0 ;

  
  if (kpis.ghTemp < -10) {           // if it ever gets to -10degC in Raleigh I'm moving to to Brazil,,,
    kpis.ghTemp = -10 ;
  }
  ghENV["mbTemp"] = kpis.mbTemp ;

  ghENV["ghTemp"] = kpis.ghTemp ;
  if (kpis.ambTemp < -10) {           // if it ever gets to -10degC in Raleigh I'm moving to to Brazil,,,
    kpis.ambTemp = -10 ;
  }

  if (kpis.tmEastTemp < -10) {           // if it ever gets to -10degC in Raleigh I'm moving to to Brazil,,,
    kpis.tmEastTemp = -10 ;
  }
  ghENV["tmEastTemp"] = kpis.tmEastTemp ;
  
  if (kpis.tmWestTemp < -10) {           // if it ever gets to -10degC in Raleigh I'm moving to to Brazil,,,
    kpis.tmWestTemp = -10 ;
  }
  ghENV["tmWestTemp"] = kpis.tmWestTemp ;

  ghENV["ambTemp"] = kpis.ambTemp ;
  ghENV["moisture0"] = kpis.moisture[0] ;
  ghENV["moisture1"] = kpis.moisture[1] ;
  ghENV["moisture2"] = kpis.moisture[2] ;
  ghENV["r0state"] = kpis.relayState[0] * 100 ;
  ghENV["r1state"] = kpis.relayState[1] * 99 ;
  ghENV["mmHg"] = kpis.mmhg ;
  ghENV["ir"] = kpis.ir ;
  ghENV["lux"] = kpis.lux ;

  ghPWR["batVdc"] = kpis.batVDC ;
  ghPWR["sysAmps"] = kpis.sysAmps ;
  ghPWR["sysAmpsRaw"] = kpis.sysAmpsRaw ;
  ghPWR["solarAmps"] = kpis.solarAmps ;
  ghPWR["solarAmpsRaw"] = kpis.solarAmpsRaw ;

  memset(buffer, 0, 1024);

  size_t n ;

  if (!mqttClient.connected()) {
    connectToMqtt();
  }

  mqttClient.beginMessage(Broker_Topic_ENV);

  n = serializeJson(ghENV, buffer) ;
  Serial.print("Serialized Environmental JSON length ");
  Serial.println(n);
  Serial.println(buffer);

  mqttClient.print(buffer) ;
  mqttClient.endMessage();

  mqttClient.beginMessage(Broker_Topic_PWR);
  n = serializeJson(ghPWR, buffer) ;
  Serial.print("Serialized Power JSON length ");
  Serial.println(n);
  Serial.println(buffer);

  mqttClient.print(buffer) ;
  mqttClient.endMessage();

  mqttClient.flush() ;
  mqttClient.stop() ;

  unsigned long tmp = millis();
  unsigned long interval = getInterval();
//  Serial.println(tmp);
//  Serial.println(msSincePwrOn);
//  Serial.println(interval);
  delayTime = interval - (tmp - msSincePwrOn) ;
  if (delayTime > interval) {
    delayTime = interval;
  }
//  Serial.println(delayTime);
  float delayTimeMin = ((float)delayTime / 60000.0);
  sprintf(buffer, "Sleeping: %.1fmin", delayTimeMin);
//  lcd.clear();
//  lcd.setCursor(0, 0) ;
//  lcd.print(buffer) ;
  
  WiFi.disconnect();
  WiFi.end();

// Send board to sleep mode
  digitalWrite(NINA_RESETN, HIGH);  // reset nina
  
  digitalWrite(LED_BUILTIN, LOW) ;

  if (delayTime > 1) {         //if we are in debug mode (10sec) this may have taken longer so just dont risk a really long delay
    LowPower.deepSleep(delayTime) ;
  }
  digitalWrite(LED_BUILTIN, HIGH) ;
}
