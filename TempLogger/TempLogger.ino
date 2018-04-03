/*
 Analog input, serial output
 Reads an analog input pinand prints the results to the serial monitor.

 The circuit:
 A0, A1 and A2 connected to the output pins of AD8495 Adafruit thermocouple modules. 
 DUT is WPC1 with Tx sent to pin 8. 
 Note: WPC1 is a 3.3V tool so serial must be level shifted. 
 Note: WPC1 Rx pin should be tied High (3.3V) 

 Author: Demian Mendez
 Last edited: 9/19/2017
 */
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
#include <SoftwareSerial.h>
SoftwareSerial mySerial(8, 9);//rx tx

// Definitions //////////////////////////////////////////////////
#define LOG_INTERVAL 1000 //mills between entries 
#define SYNC_INTERVAL 1000 // mills between calls to flush() - to write data to the card

#define ECHO_TO_SERIAL   1 // echo data to serial port
#define WAIT_TO_START    0 // Wait for serial input in setup()
  
  // Pin Assignments pins that connect to the LEDs
#define redLEDpin 2
#define greenLEDpin 3
#define clamp1Pin  A0  // AD8495 Thermocouple input
#define AmbientPin A1  // AD8495 Thermocouple input
#define clamp2Pin  A2  // AD8495 Thermocouple input


// Variable initializations/////////////////////////////////////
uint32_t syncTime = 0;    // time of last sync()

float clamp1V = 0.0;  
float clamp1avgV = 0.0;  // Sets start point for T/C to around 70°F
float clamp1TempC = 0.0;
float clamp1TempF = 0.0;

float AmbientV = 0.0;  
float AmbientavgV = 0.0;  // Sets start point for T/C to around 70°F
float AmbientTempC = 0.0;
float AmbientTempF = 0.0;

float clamp2V = 0.0;  
float clamp2avgV = 0.0;  // Sets start point for boxPin to around 70°F
float clamp2TempC = 0.0;
float clamp2TempF = 0.0;

float EMA_a = 0.0625; //0.5 For Calibration  // Constant for Smoothing filter larger value places more weight on current reading. Value should be between 0 and 1
int firstPass = 0;    // Use for the averaging function on first pass. 

  // WPC1 Variables
float DUT_Temp = 0.0;

  //Setting up the Datalogging sheild
RTC_PCF8523 rtc; // define the Real Time Clock object
  // for the data logging shield, we use digital pin 10 for the SD cs line
const int chipSelect = 10; 
  // the logging file
File logfile;

///FUNCTION PROTOTYPES  //////////////////////////////////////////
void error(char *str);
void getDutData();
long readVcc();
float getVout(int pin, double Vcc);
float ema(float EMA_A, float currentVal, float previousVal);

// SETUP /////////////////////////////////////////////////////////
void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(19200);
  mySerial.begin(9600);
  while (!Serial) {
     ; // wait for serial port to connect. Needed for native USB
  }  
  
  // Use debugging LEDs
  pinMode(redLEDpin, OUTPUT);
  pinMode(greenLEDpin, OUTPUT);

  //intialize the SD card
  Serial.print("Initializing the SD card...");
  // make sure that the default chip select pin is set to output, even if you don't use it:
  pinMode(chipSelect, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    error("Card failed, or not present");
  }
  Serial.println("card initialized.");
  
  // create a new file
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE); 
      break;  // leave the loop!
    }
  }
  
  if (! logfile) {
    error("couldnt create file");
  }
  
  Serial.print("Logging to: ");
  Serial.println(filename);

  // connect to RTC
  Wire.begin();  
  if (!rtc.begin()) {
    logfile.println("RTC failed");
#if ECHO_TO_SERIAL
    Serial.println("RTC failed");
#endif  //ECHO_TO_SERIAL
  }
  
  logfile.println("date,time,Pipe1,Ambient,WPC1,Pipe2");    
#if ECHO_TO_SERIAL
  Serial.println("date,time,Pipe1,Ambient,WPC1,Pipe2");
#endif //ECHO_TO_SERIAL
}


/// MAIN /////////////////////////////////////////////////////
void loop() {
  DateTime now;

  // delay for the amount of time we want between readings
  delay((LOG_INTERVAL -1) - (millis() % LOG_INTERVAL));

  digitalWrite(greenLEDpin, HIGH);

  // log milliseconds since starting
  uint32_t m = millis();
//  logfile.print(m);           // milliseconds since start
//  logfile.print(", ");    
#if ECHO_TO_SERIAL
//  Serial.print(m);           // milliseconds since start
//  Serial.print(", ");  
#endif

  // fetch the time
  now = rtc.now();
  // log time
//  logfile.print(now.unixtime()); // seconds since 1/1/1970
//  logfile.print(", ");
//  logfile.print('"');
  logfile.print(now.year(), DEC);
  logfile.print("/");
  logfile.print(now.month(), DEC);
  logfile.print("/");
  logfile.print(now.day(), DEC);
  logfile.print(", ");
  logfile.print(now.hour(), DEC);
  logfile.print(":");
  logfile.print(now.minute(), DEC);
  logfile.print(":");
  logfile.print(now.second(), DEC);
//  logfile.print('"');
#if ECHO_TO_SERIAL
//  Serial.print(now.unixtime()); // seconds since 1/1/1970
//  Serial.print(", ");
//  Serial.print('"');
  Serial.print(now.year(), DEC);
  Serial.print("/");
  Serial.print(now.month(), DEC);
  Serial.print("/");
  Serial.print(now.day(), DEC);
  Serial.print(", ");
  Serial.print(now.hour(), DEC);
  Serial.print(":");
  Serial.print(now.minute(), DEC);
  Serial.print(":");
  Serial.print(now.second(), DEC);
//  Serial.print('"');
#endif //ECHO_TO_SERIAL
  
  // Get the Temperature from the DUT    
  getDutData();
  
  // Get reference supply voltage for reference:
  double Vcc = readVcc()/1000.0;
  
  // read the T/C voltages:
  clamp1V   = getVout(clamp1Pin, Vcc)*2; // Time 2 multiplier for volatage divider in circuit
  AmbientV  = getVout(AmbientPin, Vcc)*2;
  clamp2V   = getVout(clamp2Pin, Vcc)*2;

  //Check to see if this is the first time through if so use the readings twice for averaging. 
  if (firstPass < 1)
  {
    clamp1avgV = clamp1V;
    AmbientavgV   = AmbientV;
    clamp2avgV = clamp2V;
    firstPass++;
  }

  //Exponential Moving Average for AD8495
  clamp1avgV    = ema(EMA_a, clamp1V, clamp1avgV);
  AmbientavgV   = ema(EMA_a, AmbientV, AmbientavgV);
  clamp2avgV    = ema(EMA_a, clamp2V, clamp2avgV);
  
  //Calculate T/C Temperature °C AD8495
  clamp1TempC    = (clamp1avgV - 1.25) / 0.005;
  AmbientTempC   = (AmbientavgV -1.25) / 0.005;
  clamp2TempC    = (clamp2avgV - 1.25) / 0.005;
  //Convert °C to °F
  clamp1TempF  = (clamp1TempC*1.8)+32;
  AmbientTempF = (AmbientTempC*1.8)+32;
  clamp2TempF  = (clamp2TempC*1.8)+32;

  logfile.print(", ");    
  logfile.print(clamp1TempF);
  logfile.print(", ");    
  logfile.print(AmbientTempF);
  logfile.print(", ");    
  logfile.print(DUT_Temp);
  logfile.print(", ");    
  logfile.print(clamp2TempF);
#if ECHO_TO_SERIAL
  Serial.print(", ");   
  Serial.print(clamp1TempF);
  Serial.print(", ");   
  Serial.print(AmbientTempF);
  Serial.print(", ");    
  Serial.print(DUT_Temp);
  Serial.print(", ");    
  Serial.print(clamp2TempF);
#endif //ECHO_TO_SERIAL

  logfile.println();
#if ECHO_TO_SERIAL
  Serial.println();
#endif // ECHO_TO_SERIAL

  digitalWrite(greenLEDpin, LOW);

  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time
  if ((millis() - syncTime) < SYNC_INTERVAL) return;
  syncTime = millis();

  // blink LED to show we are syncing data to the card & updating FAT!
  digitalWrite(redLEDpin, HIGH);
  logfile.flush();
  digitalWrite(redLEDpin, LOW);
}
///////// END MAIN /////////////////////////////////////////////////////////////////////////


// Supporting Functions
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//getDutData() 
//Description: Reads in the serial data from WPC1 and formats it to XX.X°F
//Takes: N/A
//Returns:  N/A
//===================================================================================
void getDutData(){
  int value;      //raw hex value read in serial from DUT_thermistor
  byte msb, lsb;  // bytes making up temp reading
  const byte numBytes = 12;  // Full packet size sent by WPC1
  byte rxPacket[numBytes];   // array to store the received packet
  byte endMarker = 170;
  
  if(mySerial.available()){
     mySerial.readBytesUntil(endMarker,rxPacket,numBytes);
     msb = rxPacket[9];
     lsb = rxPacket[10];
     value = (msb << 8) | lsb;
     DUT_Temp = value*0.1;   //Divide by 10 to get °F
  }
  else {
    DUT_Temp = 0;
  }
}

//readVCC() 
//Description: Measures this 1.1V reference voltage, and uses the resultant ADV value 
//             to work out what the supply voltage must be.
//Takes: N/A
//Returns:  Vcc in mV
//===================================================================================
long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1125300L / result; // Back-calculate AVcc in mV
  return result;
}

//getVout(pin,Vcc)
//Reads the analogue sensor value and converts it to a voltage. 
//Takes: Analogue pin to be measured, and supply voltage
//Returns: Vout sensor voltage
//===================================================================================
float getVout(int pin, double Vcc)
{ 
  int sensVal = analogRead(pin); // Reads in int between 0 and 1023
  float senVolt = (sensVal/1024.0)*Vcc;  //
  return senVolt;
}

//float ema(float EMA_A, float currentVal, float previousVal)
//Perfoms an exponential moving average based on input values. 
//Takes: alpha coefficient, current value and previous value. 
//Returns: Averaged value
float ema(float EMA_A, float currentVal, float previousVal)
{ 
  float EMA_S = (EMA_A*currentVal)+((1-EMA_A)*previousVal);
  return EMA_S;
}


//Error check function if we can't write to the SD card or open it. 
//===================================================================================
void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  
  // red LED indicates error
  digitalWrite(redLEDpin, HIGH);

  while(1);
}
