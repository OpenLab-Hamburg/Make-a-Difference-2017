#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);
#define SensorPin A2            //pH meter Analog output to Arduino Analog Input 0
#define Offset 0.00            //deviation compensate
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth  40    //times of collection
#include <OneWire.h>
#include <TinyGPS++.h> // Include the TinyGPS++ library
TinyGPSPlus tinyGPS; // Create a TinyGPSPlus object
#define GPS_BAUD 9600 // GPS module baud rate. GP3906 defaults to 9600.
#include <SoftwareSerial.h>
#define ARDUINO_GPS_RX 9 // GPS TX, Arduino RX pin
#define ARDUINO_GPS_TX 8 // GPS RX, Arduino TX pin
SoftwareSerial ssGPS(ARDUINO_GPS_TX, ARDUINO_GPS_RX); // Create a SoftwareSerial
#define gpsPort ssGPS  // Alternatively, use Serial1 on the Leonardo
#define SerialMonitor Serial

const int LED = 2;
const int GREEN = 3;
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex = 0;
int DS18S20_Pin = 4; //DS18S20 Signal pin on digital 4
char tmpstring[10];
OneWire ds(DS18S20_Pin);  // on digital pin 4

void setup() {
  lcd.init();//initialisation
  pinMode(LED, OUTPUT);
  SerialMonitor.begin(9600);
  gpsPort.begin(GPS_BAUD);
  Serial.begin(9600); //Baud rate: 9600
}

void loop() {
  
  // turbidity
  int sensorValue = analogRead(A0);// read the input on analog pin 0:
  float voltage = sensorValue * (5.0 / 1024.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  Serial.println(voltage); // print out the value you read:
  lcd.backlight();   // Envoi du message
  displayInfo("Turbidity", voltage, false);
  
  //conductivity
  int val = analogRead(A1);
  int conductivty = map(val, 0, 1023, 0, 14);
  displayInfo("Conductivity", conductivty, false);
  Serial.println(conductivty);
  
  //  ph
  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  static float pHValue, voltage1;
  if (millis() - samplingTime > samplingInterval)
  {
    pHArray[pHArrayIndex++] = analogRead(SensorPin);
    if (pHArrayIndex == ArrayLenth)pHArrayIndex = 0;
    voltage1 = avergearray(pHArray, ArrayLenth) * 5.0 / 1024;
    pHValue = 3.5 * voltage1 + Offset;
    samplingTime = millis();
  }
  Serial.println(pHValue);
  displayInfo("PH", pHValue, false);
  
  // temperature
  int temperature = getTemp();
  displayInfo("Temperature", temperature, true);
  Serial.println(temperature);
  
  //display
  bool cas = verification(voltage, val, pHValue, temperature);
  if (cas) {
    digitalWrite(GREEN, HIGH);
    digitalWrite(LED, LOW);
    Serial.println("true");
  }
  else {
    digitalWrite(LED, HIGH);
    digitalWrite(GREEN, LOW);
    Serial.println("false");
  }

  // print position, altitude, speed, time/date, and satellites:
  printGPSInfo();

  // "Smart delay" looks for GPS data while the Arduino's not doing anything else
  smartDelay(1000);
}

double avergearray(int* arr, int number) {
  int i;
  int max, min;
  double avg;
  long amount = 0;
  if (number <= 0) {
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if (number < 5) { //less than 5, calculated directly statistics
    for (i = 0; i < number; i++) {
      amount += arr[i];
    }
    avg = amount / number;
    return avg;
  } else {
    if (arr[0] < arr[1]) {
      min = arr[0]; max = arr[1];
    }
    else {
      min = arr[1]; max = arr[0];
    }
    for (i = 2; i < number; i++) {
      if (arr[i] < min) {
        amount += min;      //arr<min
        min = arr[i];
      } else {
        if (arr[i] > max) {
          amount += max;  //arr>max
          max = arr[i];
        } else {
          amount += arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount / (number - 2);
  }//if
  return avg;
}
int getTemp() {
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
    //no more sensors on chain, reset search
    ds.reset_search();
    return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
    Serial.print("Device is not recognized");
    return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad

  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  int TemperatureSum = tempRead / 16; //conversion en int

  return TemperatureSum;
}
void displayInfo (String title, float value, bool isInteger ) {
  String valueString;
  if (isInteger)
  {
    valueString = String(int(value));
  }
  else
  {
    valueString = String(value);
  }
  int margin = (16 - title.length()) / 2;
  lcd.setCursor(margin, 0);
  lcd.print(title);
  int valueMargin = (16 - valueString.length()) / 2;
  lcd.setCursor(valueMargin, 1);
  lcd.print(valueString);
  delay(2000);
  lcd.clear();

}
bool verification(float voltage, int conductivity , float pHValue, int temperature )
{
  bool qual;
  int i = 0;
  int tempMin = 15, tempMax = 25;
  int phMin = 6, phMax = 7.5;
  int voltageMin = 4, voltageMax = 5;
  int conductivityMin = 0.005 , conductivityMax = 0.05;
  if (tempMin < temperature && tempMax > temperature)
  {
    i++;
  }
  if (phMin < pHValue &&  phMax > pHValue)
  {
    i++;
  }
  if (voltageMin < voltage &&  voltageMax > voltage)//turbidity value
  {
    i++;
  }
  if (conductivityMin < conductivity &&  conductivityMax > conductivity)//conductivity value
  {
    i++;
  }
  if (i == 4) {
    qual = true;
  }
  else {
    qual = false;
  }
  return qual;
}


void printGPSInfo()
{
  // Print latitude, longitude, altitude in feet, course, speed, date, time,
  // and the number of visible satellites.
  SerialMonitor.println(tinyGPS.location.lat(), 6);
  SerialMonitor.println(tinyGPS.location.lng(), 6);
  //  SerialMonitor.print("Alt: "); SerialMonitor.println(tinyGPS.altitude.feet());
  //  SerialMonitor.print("Course: "); SerialMonitor.println(tinyGPS.course.deg());
  //  SerialMonitor.print("Speed: "); SerialMonitor.println(tinyGPS.speed.mph());
//  SerialMonitor.print("Date: "); printDate();
//  SerialMonitor.print("Time: "); printTime();
  //  SerialMonitor.print("Sats: "); SerialMonitor.println(tinyGPS.satellites.value());
  SerialMonitor.println();
}

// This custom version of delay() ensures that the tinyGPS object
// is being "fed". From the TinyGPS++ examples.
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    // If data has come in from the GPS module
    while (gpsPort.available())
      tinyGPS.encode(gpsPort.read()); // Send it to the encode function
    // tinyGPS.encode(char) continues to "load" the tinGPS object with new
    // data coming in from the GPS module. As full NMEA strings begin to come in
    // the tinyGPS library will be able to start parsing them for pertinent info
  } while (millis() - start < ms);
}

// printDate() formats the date into dd/mm/yy.
void printDate()
{
  SerialMonitor.print(tinyGPS.date.day());
  SerialMonitor.print("-");
  SerialMonitor.print(tinyGPS.date.month());
  SerialMonitor.print("-");
  SerialMonitor.println(tinyGPS.date.year());
}

// printTime() formats the time into "hh:mm:ss", and prints leading 0's
// where they're called for.
void printTime()
{
  String currentTime = String(tinyGPS.time.hour()) + String(tinyGPS.time.minute()) + String(tinyGPS.time.second());
  SerialMonitor.print(currentTime);
 
}


