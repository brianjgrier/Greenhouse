//
// This is to control a small greenhousse/coldframe
// Automatically water the plants when moisture level drops below setpoint

#include <OneWire.h>
#include <DallasTemperature.h>

#include <Wire.h>
#include <SPI.h>
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
int status = WL_IDLE_STATUS;       //connection status

WiFiClient wifiClient ;
MqttClient mqttClient(wifiClient);

#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 column and 2 rows


const char Broker_Host[] = "sds-orange-pi5" ;
const int Broker_Port = 1883 ;
const char Broker_Topic[] = "/raleigh/home/greenhouse" ;

const int VALVE_OPEN_MAX_TIMER_LIMIT = 20 ;
const int VAVLE_CLOSED_MIN_TIMER_LIMIT = 30 ;
const int NUMBER_OF_VALVES = 2 ;
const unsigned long ONE_SECOND      =     1000 ;
const unsigned long TEN_SECONDS     =    10000 ;
const unsigned long ONE_MINUTE      =    60000 ;
const unsigned long FIVE_MINUTES    =   300000 ;
const unsigned long TEN_MINUTES     =   600000 ;
const unsigned long FIFTEEN_MINUTES =   900000 ;
const unsigned long TWENTY_MINUTES  =  1200000 ;

const int HIGH_MOISTURE = 80 ;
const int LOW_MOISTURE = 35 ;

//const int voltageSensor = A4;
//const int currentSensor = A6;
//
// The RP2040 Connect does not like assigning A4 and A6 to const int so...
//

#define voltageSensor A4
#define currentSensor A6

long msSincePwrOn ;

// Pin to put interval to 10 seconds
int intervalCheck = 6 ;

// The Digital Pin definitions
int valve1StatePin = 2;
int valve1ControlPin = 3;

// Initially valve 2 & 3 are not used at this time
int valve2StatePin = 4;
int valve2ControlPin = 5;

int tempSensorsDataPin = 8;  // DB18B20 one-wire-bus

// The Analog Pin definitions for ground moisture sensors

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

struct MoistureSenfor_Info {
  int  sensorPin ;
  long sensorMoisture ;
  long sensorMap[4] ;
  int  controlValve ;
} ;

struct MoistureSenfor_Info moistureSensor[3] =
    {{A0, -1, {3370, 2860, 0, 100},  0},
     {A1, -1, {3370, 2860, 0, 100}, -1},
     {A2, -1, {3370, 2860, 0, 100},  1}} ;

// Greenhouse datapoints
float greenhouseTemp;
float ambientTemp;
float lux;

float batVolt;

OneWire oneWire(tempSensorsDataPin);      // setup a oneWire instance
DallasTemperature tempSensors(&oneWire);  // pass oneWire to DallasTemperature library

struct temp_sensor_struct {
  char id[32];
  float temp;
  uint8_t macId[8];
};

struct temp_sensor_struct tempSensorAddrs[2] = { { "Greenhouse", 0.0, { 0x28, 0xCA, 0x99, 0x58, 0xD4, 0xE1, 0x3C, 0xE5 } },  // no tape
                                                 { "Ambient", 0.0, { 0x28, 0x96, 0x53, 0x58, 0xD4, 0xE1, 0x3C, 0x29 } } };   // Blue tape

int numTempSensors = sizeof(tempSensorAddrs) / sizeof(tempSensorAddrs[0]) ;

struct valve_struct {
  int statePin;
  int controlPin;
  bool state;
  long transitionDelay ;
};

struct valve_struct valves[2] = { { valve1StatePin, valve1ControlPin, false, 0l},
                                  { valve2StatePin, valve2ControlPin, false, 0l} };


struct {
  float     mbTemp ;
  float     ghTemp ;
  float     ambTemp ;
  int       moisture[3] ;
  int       valveState[2] ;
  int       mmhg ;
  uint16_t  ir ;
  uint16_t  lux ;
  float     batVDC ;
  float     sysAmps;
} kpis ;

void setValve(int valve, int desiredState) {
  int valveState ;

  if ( valves[valve].transitionDelay <= 0 ) {
    valveState = digitalRead(valves[valve].statePin) ;
    if (valveState != desiredState) {
     // Togggle the clock on the 74HC74 to change the state
      digitalWrite(valves[valve].controlPin, LOW) ;     // Make sure it is LOW because state changes on LOW-HIGH transition
      delayMicroseconds(100) ;
      digitalWrite(valves[valve].controlPin, HIGH) ;
      delayMicroseconds(100);
      digitalWrite(valves[valve].controlPin, LOW) ;
      valveState = digitalRead(valves[valve].statePin) ;
      if (valveState != desiredState) {
        Serial.print("Valve #") ;
        Serial.print(valve) ;
        Serial.print(" - Failed to ");
        if (desiredState = 0) {
          Serial.println("Close") ;
        } else {
          Serial.println("Open") ;
        }
      }
//      valves[valve].transitionDelay = TWENTY_MINUTES ;
      valves[valve].transitionDelay = 3 ;
    }
    valves[valve].state = valveState ;   //Save the surrent valve state
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
  //WiFiDrv::digitalWrite(27, HIGH); //Turn On Blue light
  lcd.clear();                 // clear display
  lcd.setCursor(0, 0);         // move cursor to   (0, 0)
  lcd.print("Connect to WiFi.");        // print message at (0, 0)

  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    Serial.println(status);
 //   if (status != WL_CONNECTED) {
 //     WiFiDrv::digitalWrite(27, LOW); //Turn Off Blue light
 //     delay(100) ;
 //     WiFiDrv::digitalWrite(27, HIGH); //Turn On Blue light
 //   }
    // wait 10 seconds for connection:
    delay(10000);
  }
//  WiFiDrv::digitalWrite(27, LOW); //Turn Off Blue light
  WiFi.lowPowerMode();
  lcd.clear();                 // clear display
  lcd.setCursor(0, 0);         // move cursor to   (0, 0)
  lcd.print("WiFi: Connected") ;
}

int getInterval() {
  int result = 0 ;
  if (digitalRead(intervalCheck) == LOW) {
    Serial.println("TEN_SECONDS") ;
    result = TEN_SECONDS ;
  } else {
    Serial.println("FIVE_MINUTES") ;
    result = FIVE_MINUTES ;
  }
  return result ;
}

void connectToMqtt() {
  int limit = 60 ;

  Serial.print("Attempting to connect to the MQTT broker: ");
  lcd.clear();                 // clear display
  lcd.setCursor(0, 0);         // move cursor to   (0, 0)
  lcd.print("Connect to MQTT");        // print message at (0, 0)

  Serial.print(Broker_Host);
  Serial.print(", on port -");
  Serial.println(Broker_Port);
//  WiFiDrv::digitalWrite(25, HIGH); //Turn On Red light
  while (!mqttClient.connect(Broker_Host, Broker_Port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    limit-- ;
    if (limit <= 0 ) {
      lcd.clear();
      lcd.setCursor(0, 0) ;
      lcd.print("Reseting board");
      NVIC_SystemReset(); 
      return ;
    }
    delay(1000) ;
  }
  mqttClient.setKeepAliveInterval(TEN_MINUTES) ;
  mqttClient.setConnectionTimeout(FIFTEEN_MINUTES) ;
//  WiFiDrv::digitalWrite(25, LOW); //Turn Off Red light
  lcd.clear();
  lcd.setCursor(0, 0);         // move cursor to   (0, 0)
  lcd.print("MQTT: Connected");        // print message at (0, 0)

  return ;
}


void setup() {
  // put your setup code here, to run once:
  msSincePwrOn = 0 ;

  Serial.begin(9600);
  delay(2000);

  lcd.init();         // initialize the lcd
  lcd.backlight();    // Turn on the backlight (Yeah it sucks energy but...)

  pinMode(intervalCheck, INPUT_PULLUP) ;

  //
  // Set up the digital pins for the water valve
  for (int i = 0; i < 2; i++) {
    pinMode(valves[i].statePin, INPUT) ;
    pinMode(valves[i].controlPin, OUTPUT) ;
    valves[i].transitionDelay = 0 ; // Do not let the valves change for until the seconf time through the loop
  }

  for (int i = 0; i < 3; i++){
    pinMode(moistureSensor[i].sensorPin, INPUT) ;

  } ;
  
  pinMode(voltageSensor, INPUT) ;      // Voltage Senosr input
  pinMode(currentSensor, INPUT) ;      // Current Sensor Input

  //analogReference(AR_VDD); 
  analogReadResolution(12);       // only if on an RP2040 or MKR board

//
// Make sure the valves are closed.
// Do not let them transition until the second time through 'loop'
//
  setValve(0, 0) ;
  setValve(1, 0) ;

// Set up oversampling and filter initialization
  bmp.begin_I2C() ;
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
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

//
// Now cycle the LEDs
  WiFiDrv::digitalWrite(27, HIGH); //Turn On Blue light
  delay(125);
  WiFiDrv::digitalWrite(27, LOW); //Turn Off Blue light
  delay(125);
  WiFiDrv::digitalWrite(26, HIGH); //Turn On Red light
  delay(125);
  WiFiDrv::digitalWrite(26, LOW); //Turn Off Red light
  delay(125);
  WiFiDrv::digitalWrite(25, HIGH); //Turn On Green light
  delay(125);
  WiFiDrv::digitalWrite(25, LOW); //Turn Off Green light
  delay(125);

  enable_WiFi();
  connect_WiFi();

  printWifiStatus();

  connectToMqtt() ;     // Just to make sure we can
  mqttClient.stop() ;   // disconnect and we will pick it up in LOOP

  WiFiDrv::digitalWrite(26, HIGH); //Turn On Green light
//
// Initialize the data collection for mqtt
//
  kpis.ambTemp = -1.0 ;
  kpis.ghTemp = -1.0 ;
  kpis.mbTemp = -1.0 ;
  for (int i=0; i < 3; i++) kpis.moisture[i] = 0 ;
  for (int i=0; i < 2; i++) kpis.valveState[i] = 0 ;
  kpis.ir = 0 ;
  kpis.lux = 0 ;
  kpis.mmhg = 0 ;
  kpis.sysAmps = 0.0 ;
  kpis.batVDC = 0.0 ;


  Serial.println("Setup is complete");
  Serial.flush();
  delay(getInterval());
}

void loop() {
  long delayTime ;
  char outString[80] ;
//
// reduce the valve no change interval by one period
//
  for ( int i = 0; i < 2; i++){
    valves[i].transitionDelay = valves[i].transitionDelay - 1 ;
    if (valves[i].transitionDelay < 0) {
      valves[i].transitionDelay = 0;
    }
  }

  // put your main code here, to run repeatedly:
  long currMs ;

  float acsValue ;
  float acsCum ;
  float acsVoltage ;
  float acsCurrent ;

  msSincePwrOn = millis();
  // Get the LUX reading

  // Get the Inside Temperature
  tempSensors.requestTemperatures();

  kpis.ghTemp = tempSensors.getTempC(tempSensorAddrs[0].macId) ;
  kpis.ambTemp = tempSensors.getTempC(tempSensorAddrs[1].macId) ;

  // Get the soil moisture readings

  for (int i = 0; i < 3; i++){
    long moisturePercentage ;
    moistureSensor[i].sensorMoisture = (long)analogRead(moistureSensor[i].sensorPin) ;
    moisturePercentage = map(moistureSensor[i].sensorMoisture,
                         moistureSensor[i].sensorMap[0],
                         moistureSensor[i].sensorMap[1],
                         moistureSensor[i].sensorMap[2],
                         moistureSensor[i].sensorMap[3]) ;
    if ( moisturePercentage < 0 ) {
      moisturePercentage = 0 ;
    } ;

    kpis.moisture[i] = moisturePercentage ;
    if ( moisturePercentage < LOW_MOISTURE ) {
      if (moistureSensor[i].controlValve != -1) 
        setValve(moistureSensor[i].controlValve, 1);
    }
    if ( moisturePercentage > HIGH_MOISTURE ) {
      if (moistureSensor[i].controlValve != -1) 
        setValve(moistureSensor[i].controlValve, 0);
    }
  }

  for (int i = 0; i < NUMBER_OF_VALVES; i++){
    kpis.valveState[i] = digitalRead(valves[i].statePin) ;
  }


  kpis.batVDC = map(analogRead(voltageSensor), 0, 4095, 0.0, 20.0) ;
 
  acsCum = 0.0 ;
  for (int i=0; i < 50; i++) {
    acsCum += analogRead(currentSensor) ;
    delay(5);
  }
  {
    acsValue = acsCum / 50 ;
    acsVoltage = acsValue*(3.3/4095.0) ;
    acsCurrent = (acsVoltage - (1.65 - 0.055)) / 0.185 ;
  }

  Serial.println(acsValue,3) ;
  Serial.println(acsVoltage,3);
  Serial.println(acsCurrent,3);

  kpis.sysAmps = acsCurrent ;


  if (! bmp.performReading()) {
    kpis.mbTemp = -1.0 ;    // signify a read error
    kpis.mmhg = -1 ;
  } else {
    kpis.mbTemp = bmp.temperature ;
    kpis.mmhg = (int)((bmp.pressure / 100.0) / 1.33322) ;
  }

  advancedRead(&tslData) ;

  kpis.ir = tslData.ir ;
  kpis.lux = tsl.calculateLux(tslData.full, tslData.ir) ;

  DynamicJsonDocument root(2048) ;

  root["mbTemp"] = kpis.mbTemp ;
  if (kpis.ghTemp < -10) {           // if it ever gets to -10degC in Raleigh I'm moving to to Brazil,,,
    kpis.ghTemp = -10 ;
  }
  root["ghTemp"] = kpis.ghTemp ;
  if (kpis.ambTemp < -10) {           // if it ever gets to -10degC in Raleigh I'm moving to to Brazil,,,
    kpis.ambTemp = -10 ;
  }
  root["ambTemp"] = kpis.ambTemp ;
  root["moisture0"] = kpis.moisture[0] ;
  root["moisture1"] = kpis.moisture[1] ;
  root["moisture2"] = kpis.moisture[2] ;
  root["v0state"] = kpis.valveState[0] * 100 ;
  root["v1state"] = kpis.valveState[1] * 99 ;
  root["mmhg"] = kpis.mmhg ;
  root["ir"] = kpis.ir ;
  root["lux"] = kpis.lux ;
  root["batVdc"] = kpis.batVDC ;
  root["sysAmps"] = kpis.sysAmps ;

  char buffer[2048] ;
  size_t n = serializeJson(root, buffer) ;

  if (!mqttClient.connected()) {
    connectToMqtt();
  }

  mqttClient.beginMessage(Broker_Topic);
  mqttClient.print(buffer) ;
  mqttClient.endMessage();
  mqttClient.flush() ;
  mqttClient.stop() ;

  Serial.flush() ;

//  WiFiDrv::digitalWrite(26, HIGH); //Turn On Green light
//  delay(250) ;
//  WiFiDrv::digitalWrite(26, LOW); //Turn Off Green light

  lcd.clear();
  lcd.setCursor(0, 0);         // move cursor to   (0, 0)

  delayTime = getInterval() - (millis() - msSincePwrOn) ;
  float delayTimeMin = ((float)delayTime / 60000.0);
  sprintf(buffer, "Sleeping: %.1f Min", delayTimeMin);    
  lcd.print(buffer) ;
  
  if (delayTime > 1) {         //if we are in debug mode (10sec) this may have taken longer so just dont risk a really long delay
    delay(delayTime) ;
  }

}
