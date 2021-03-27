//==========================================================
//
// Change History:
// v1 First cut from existing other code.  Using NodeMCU 8266 Board.
// v2 Updated and made better note of library dependencies.
//
//==========================================================
//  NodeMCU GPIO Pins:
//  GPIO Pin   I/O Index Number
//  GPIO0       D3      Using for WiFi Reset Button
//  GPIO1       D10
//  GPIO2       D4
//  GPIO3       D9
//  GPIO4       D2      Using for I2C SDA
//  GPIO5       D1      Using for I2C SCL
//  GPIO6       N/A
//  GPIO7       N/A
//  GPIO8       N/A
//  GPIO9       D11
//  GPIO10      D12
//  GPIO11      N/A
//  GPIO12      D6
//  GPIO13      D7
//  GPIO14      D5
//  GPIO15      D8
//  GPIO16      D0      Built-in LED
//==========================================================
//  Some or much of this code is from BME280 I2C Test.ino
//  This code shows how to record data from the BME280 environmental sensor
//  using I2C interface. This file is an example file, part of the Arduino
//  BME280 library.
//  Copyright (C) 2016  Tyler Glenn
//  
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//  
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//  
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
//  
//  Written: Dec 30 2015.
//  Last Updated: Sep 19 2016.
//  
//  Connecting the BME280 Sensor:
//  Sensor              ->  Board
//  -----------------------------
//  Vin (Voltage In)    ->  3.3V
//  Gnd (Ground)        ->  Gnd
//  SDA (Serial Data)   ->  A4 on Uno/Pro-Mini, 20 on Mega2560/Due, 2 Leonardo/Pro-Micro
//  SCK (Serial Clock)  ->  A5 on Uno/Pro-Mini, 21 on Mega2560/Due, 3 Leonardo/Pro-Micro
//==========================================================
//  WiFi Manager: How It Works
//
//  when your ESP starts up, it sets it up in Station mode and tries to connect to a previously saved Access Point
//  if this is unsuccessful (or no previous network saved) it moves the ESP into Access Point mode and spins up a 
//  DNS and WebServer (default ip 192.168.4.1)
//  using any wifi enabled device with a browser (computer, phone, tablet) connect to the newly created Access Point
//  because of the Captive Portal and the DNS server you will either get a 'Join to network' type of popup or get 
//  any domain you try to access redirected to the configuration portal
//  choose one of the access points scanned, enter password, click save
//  ESP will try to connect. If successful, it relinquishes control back to your app. If not, reconnect to AP and reconfigure.


//==== Password Includes ===================================
#include "BME280_Weather-ThingSpeak-WifiManager-LCD.h"
//  unsigned long myChannelID = xxxxx;
//  const char * myWriteAPIKey = "xxxxx";


//===============================================================
// LCD Code includes
// Created by Bill Perry 2016-07-02
// This sketch is for LCDs with PCF8574 or MCP23008 chip based backpacks
// WARNING:
// Use caution when using 3v only processors like arm and ESP8266 processors
// when interfacing with 5v modules as not doing proper level shifting or
// incorrectly hooking things up can damage the processor.
// 
// Sketch will print "Hello, World!" on top row of lcd
// and will print the amount of time since the Arduino has been reset
// on the second row.
//
// If initialization of the LCD fails and the arduino supports a built in LED,
// the sketch will simply blink the built in LED.
//
// NOTE:
// If the sketch fails to produce the expected results, or blinks the LED,
// run the included I2CexpDiag sketch to test the i2c signals and the LCD.

#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header

//==== BME280 Global Variables =============================
  //  Recommended Modes -
  //  Based on Bosch BME280I2C environmental sensor data sheet.
  //
  //  Weather Monitoring :
  //    forced mode, 1 sample/minute
  //    pressure ×1, temperature ×1, humidity ×1, filter off
  //    Current Consumption =  0.16 μA
  //    RMS Noise = 3.3 Pa/30 cm, 0.07 %RH
  //    Data Output Rate 1/60 Hz
  BME280I2C::Settings settings(
    BME280::OSR_X1,
    BME280::OSR_X1,
    BME280::OSR_X1,
    BME280::Mode_Forced,
    BME280::StandbyTime_1000ms,
    BME280::Filter_Off,
    BME280::SpiEnable_False,
    BME280I2C::I2CAddr_0x76       // I2C address. I2C specific.
  );
  BME280I2C bme(settings);  //Use the defaults
  bool metric = true;
  float temperature(NAN), humidity(NAN), pressure(NAN);
//====  END Includes =======================================

//==== General Defines =====================================
#define SERIAL_BAUD 115200
#define WIFIRESETBUTTON D3
//==== END Defines =========================================



//==== BME280 WiFiManager Global Variables =================
  Ticker ticker;

//==== NTP  Global Variables ===============================
  unsigned int localPort = 2390;            // local port to listen for UDP packets
  
  // Don't hardwire the IP address or we won't get the benefits of the pool.
  // Lookup the IP address for the host name instead
  // IPAddress timeServer(129, 6, 15, 28);  // time.nist.gov NTP server
  IPAddress timeServerIP;                   // time.nist.gov NTP server address
  const char* ntpServerName = "time.nist.gov";
  const int NTP_PACKET_SIZE = 48;           // NTP time stamp is in the first 48 bytes of the message
  const long TZOFFSET = 10 * 3600;          // TNS: Rough aproach to Timezone offset in seconds
                                            // for Austrlian EST ie GMT+10 hours
  byte packetBuffer[ NTP_PACKET_SIZE];      //buffer to hold incoming and outgoing packets
  
  WiFiUDP udp;                              // A UDP instance to let us send and receive packets over UDP

  struct MYTIME{
    int h;
    int m;
    int s; 
  };
  MYTIME mt = {0, 0, 0};                    // Use this to figure out when to have the display on.
  MYTIME ont = {6,0,0};
  MYTIME offt = {20,0,0};

//====  ThingSpeak Global Variables ========================
  WiFiClient client;                        // Need this for ThingSpeak?
  uint32_t delayMS;
  // Set up some time variables
  // 2^32 -1 - gives me about 24 days before it rolls over.
  unsigned long oldTime, newTime;
  // 5 minutes = 1000 x 60 x 5 = 300000
  #define THINGSPEAKDELAY 300000            // This is how long between measures.

//====  LCD Global Variables ===============================
  hd44780_I2Cexp lcd(0x27); // declare lcd object: auto locate & config exapander chip
  
  // LCD geometry
  const int LCD_ROWS = 2;
  const int LCD_COLS = 16;

//====  WiFiManager Variables ==============================
  // Really not pretty passing around globals like this.  Should review and do as pointers etc.
  WiFiManager wifiManager;
  
//==== END Global Variables ================================


//==========================================================
//===== FUNCTIONS ==========================================
//==========================================================
// ThingSpeak uploader routine
// PRE:  Uses globals of temperature, humidity.
// POST: sent current_temp; current_humidity to ThingSpeak "myChannelID"
//==========================================================
void writeThingSpeak(){
  // LED ON to indicate we are handling a request
  digitalWrite(BUILTIN_LED, LOW);  
  // Write to ThingSpeak. There are up to 8 fields in a channel, allowing you to store up to 8 different
  // pieces of information in a channel.  
  // Set the fields to be set in ThingSpeak
  ThingSpeak.setField(1,temperature);
  ThingSpeak.setField(2,humidity);

  // Write the fields to ThingSpeak
  ThingSpeak.writeFields(myChannelID, myWriteAPIKey);
  //  Possible response codes:
  //  200: OK / Success
  //  404: Incorrect API key (or invalid ThingSpeak server address)
  //  -101: Value is out of range or string is too long (> 255 characters)
  //  -201: Invalid field number specified
  //  -210: setField() was not called before writeFields()
  //  -301: Failed to connect to ThingSpeak
  //  -302: Unexpected failure during write to ThingSpeak
  //  -303: Unable to parse response
  //  -304: Timeout waiting for server to respond
  //  -401: Point was not inserted (most probable cause is the rate limit of once every 15 seconds)  
  String message = ThingSpeak.readStringField(myChannelID, 1);
  int resultCode = ThingSpeak.getLastReadStatus();
  if(resultCode == 200  || resultCode == 400)
  {
    Serial.print("Latest message is: "); 
    Serial.println(message);
    Serial.print("Sent to ThingSpeak:: ");
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print("  Humidity: ");
    Serial.println(humidity);  }
  else
  {
    Serial.print("Error reading message.  Status was: "); 
    Serial.println(resultCode);
  }
  // LED OFF to indicate we are finished
  digitalWrite(BUILTIN_LED, HIGH);    
}

//==========================================================
// WiFiManager Main Function
// PRE: Generalised function to reset the WiFi if needed
//      ClearWIFI = true will force clearing of the WIFI settings
//
//      Fetches ssid and password that has been previously collected and tries to connect
//      if it does not connect it starts an access point with the specified name
//      here  "AutoConnectAP"
//      and goes into a blocking loop awaiting configuration.
//==========================================================
void configureWIFI(boolean ClearWIFI){
  // start ticker with 0.5 because we start in AP mode and try to connect
  ticker.attach(0.6, tick);  

  if (ClearWIFI){
    // reset settings
    Serial.println("Clearing WIFI settings");
    wifiManager.resetSettings();
    ESP.reset();
  }


  // set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);

  // fetches ssid and pass and tries to connect
  // if it does not connect it starts an access point with the specified name
  // here  "AutoConnectAP"
  // and goes into a blocking loop awaiting configuration
  if(!wifiManager.autoConnect()) {
    Serial.println("failed to connect and hit timeout");
    // reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  } 

  //if you get here you have connected to the WiFi
  Serial.println("Connected...");
  ticker.detach();
  //keep LED on
  digitalWrite(BUILTIN_LED, LOW);
}

//==========================================================
void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
  //entered config mode, make led toggle faster
  ticker.attach(0.2, tick);
}

//==========================================================
// Simple LED Toggle function used in WiFiManager
void tick(){
  //toggle state
  int state = digitalRead(BUILTIN_LED);  // get the current state of GPIO1 pin
  digitalWrite(BUILTIN_LED, !state);     // set pin to the opposite state
}




//==========================================================
void takeBME280Reading(void){
  // TNS - Notionally we are in "forced mode" which means we need to trigger the read then wait 8ms
  //bme.setMode(0x01);
  delay(10);
 
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_hPa);
 
  // Parameters: (float& pressure, float& temp, float& humidity, bool celsius = false, uint8_t pressureUnit = 0x0) 
  bme.read(pressure, temperature, humidity, tempUnit, presUnit);   
  
}



//==========================================================
void printBME280Data(Stream* client){
  client->print("Temp: ");
  client->print(temperature);
  client->print("°"+ String(metric ? 'C' :'F'));
  client->print("\t\tHumidity: ");
  client->print(humidity);
  client->print("% RH");
  client->print("\t\tPressure: ");
  client->print(pressure);
  client->print(" atm");
}


//==========================================================
void printBME280CalculatedData(Stream* client){
  EnvironmentCalculations::AltitudeUnit envAltUnit  =  EnvironmentCalculations::AltitudeUnit_Meters;
  EnvironmentCalculations::TempUnit     envTempUnit =  EnvironmentCalculations::TempUnit_Celsius;

  float absHum = EnvironmentCalculations::AbsoluteHumidity(temperature, humidity, envTempUnit);

  client->print("\t\tHeat Index: ");
  float heatIndex = EnvironmentCalculations::HeatIndex(temperature, humidity, envTempUnit);
  client->print(heatIndex);
  client->print("°"+ String(envTempUnit == EnvironmentCalculations::TempUnit_Celsius ? "C" :"F"));

  client->print("\t\tAbsolute Humidity: ");
  client->println(absHum);
}

//==========================================================
void displayLCDBME280Data(void){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp:");
  lcd.print(temperature);
  lcd.print("%");

  lcd.setCursor(0, 1);
  lcd.print("Humid:");
  lcd.print(humidity);
  lcd.print("% RH");  
}

//==========================================================
void getNTPTime()
{
  //get a random server from the pool
  WiFi.hostByName(ntpServerName, timeServerIP); 

  sendNTPpacket(timeServerIP); // send an NTP packet to a time server
  // wait to see if a reply is available
  delay(1000);
  
  int cb = udp.parsePacket();
  if (!cb) {
    Serial.println("no packet yet");
  }
  else {
    Serial.print("packet received, length=");
    Serial.println(cb);
    // We've received a packet, read the data from it
    udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    Serial.print("Seconds since Jan 1 1900 = " );
    Serial.println(secsSince1900);

    // now convert NTP time into everyday time:
    Serial.print("Unix time = ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears + TZOFFSET;
    // print Unix time:
    Serial.println(epoch);


    // print the hour, minute and second:
    Serial.print("The local time is ");       // UTC is the time at Greenwich Meridian (GMT)
    Serial.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
    Serial.print(':');
    if ( ((epoch % 3600) / 60) < 10 ) {
      // In the first 10 minutes of each hour, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
    Serial.print(':');
    if ( (epoch % 60) < 10 ) {
      // In the first 10 seconds of each minute, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.println(epoch % 60); // print the second
    // Keep track of time for screen blanking
    mt.h = (epoch  % 86400L) / 3600;
    mt.m = (epoch  % 3600) / 60;
    mt.s = epoch % 60;
  }
}

//==========================================================
// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address)
{
  Serial.println("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;            // Stratum, or type of clock
  packetBuffer[2] = 6;            // Polling Interval
  packetBuffer[3] = 0xEC;         // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}

//==========================================================
// fatalError() - loop & blink an error code
void fatalError(int ecode)
{
  hd44780::fatalError(ecode); // does not return
}
/* ==== END Functions ==== */

//========================================================== 
// Check for day period when we do want the display active.
// Globals we use are MYTIME mt, ont, offt
// Disable this by setting ont == offt
bool isDisplayTime(void)
{
  // Convert the structs to simple integers
  unsigned long mtsecs = mt.h*60*60 + mt.m*60 +mt.s;
  unsigned long ontsecs = ont.h*60*60 + ont.m*60 +ont.s;
  unsigned long offtsecs = offt.h*60*60 + offt.m*60 +offt.s;

  String t = "Daylight check: mt " + String(mtsecs) + ":ont " + String(ontsecs) + ":offt " + String(offtsecs);
  Serial.println(t);  

  if (ontsecs == offtsecs){return true;}
  if (mtsecs > ontsecs & mtsecs < offtsecs) {return true;}

  // OK we should have the display off
  Serial.println("Display in night mode.");
  return false;
}



//==========================================================
//===== SETUP ==============================================
//==========================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  while(!Serial) {} // Wait

  // Start the BME280 Sensor
  Wire.begin();
  while(!bme.begin()){
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }

  // starttime = millis();//get the current time;

  // set a button input to know when to reset the WiFi
  pinMode(WIFIRESETBUTTON, INPUT);
  
  // set led pin as output
  pinMode(BUILTIN_LED, OUTPUT);

  // initialize LCD with number of columns and rows: 
  if( lcd.begin(LCD_COLS, LCD_ROWS))
  {
    // begin() failed so blink the onboard LED if possible
    fatalError(1); // this never returns
  }
  // Print a message to the LCD
  lcd.print("Starting!");

    
  // This loops until the WIFI is configured
  configureWIFI(false);

  lcd.backlight();
  lcd.clear();
  lcd.print("Connected to:");
  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP()); 

  // StartUDP handler for NTP
  Serial.println("Starting UDP");
  udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(udp.remotePort());
  // Wait so we can read it!
  delay(5000);  

  // Instantiate ThingSpeak
  ThingSpeak.begin(client);

  // Note when we started
  oldTime = millis();

  // Take some initial readings to be sure the BME280 is being seen
  takeBME280Reading();
  printBME280Data(&Serial);
  printBME280CalculatedData(&Serial); 

  displayLCDBME280Data();
}
/* ==== END Setup ==== */

//==========================================================
//===== LOOP ===============================================
//==========================================================
void loop() {
  // LED OFF
  digitalWrite(BUILTIN_LED, HIGH);
  lcd.noBacklight();
  
  // Measure the temperature etc  every 5 minutes and send to ThingSpeak
  newTime = millis();

  if((newTime - oldTime) > THINGSPEAKDELAY){
    oldTime = newTime;
    
    getNTPTime();
    
    // BME code, and we'll have a nap at the end
    takeBME280Reading();
    printBME280Data(&Serial);
    printBME280CalculatedData(&Serial);
    if (isDisplayTime()){
      displayLCDBME280Data();
    }else{
      lcd.noBacklight();
    }
    // Write it to the CLoud
    writeThingSpeak();

  }

  // Check to see if the WiFi reset button is pressed (LOW)
  // I'm sure this can be done neater.
  if (!digitalRead(WIFIRESETBUTTON)){
    // OK the reset button is set
    Serial.println("WIFIRESETBUTTON pressed");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi Reset");
    lcd.setCursor(0, 1);
    lcd.print("See 192.168.4.1");
    configureWIFI(true);
  }
}
/* ==== End Loop ==== */
