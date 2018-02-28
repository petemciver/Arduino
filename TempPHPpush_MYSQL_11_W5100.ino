// New Weather station 
// with routines to push data to MYSQL database
// integrating I2C routines for wind 
// version 11 with LED13 indicator
// for W5100 shield

#include <Wire.h>
#include <Event.h>
#include <Timer.h>
#include <EasyTransferI2C.h>
#include <Ethernet.h> // Used for Ethernet
#include <SPI.h>
#include <OneWire.h>     // Used for temperature sensor(s)
#include <Adafruit_BMP085.h>

#define DEBUG            // Remove this line if you don't want serial output 
//
//create object for EasyTransfer
EasyTransferI2C ET; 
struct SEND_DATA_STRUCTURE{
  //put variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int etgust;
  int etwindspeed;
  int etwindforce;
  int windcount;
};
//
//give a name to the group of data
SEND_DATA_STRUCTURE mydata;
//
//define slave i2c address
#define I2C_SLAVE_ADDRESS 9

// Wind dir / speed
//
//Uncomment one of the following depending on the orientation of the encoder
//1. Shaft pointing up array
//char* Dirs[]={"N  ", "NNE", "NE ", "ENE", "E  ", "ESE", "SE ", "SSE", "S  ", "SSW", "SW ", "WSW", "W  ", "WNW", "NW ", "NNW"};
//2. Shaft pointing down array
char* Dirs[]={"N  ", "NNW", "NW ", "WNW", "W  ", "WSW", "SW ", "SSW", "S  ", "SSE", "SE ", "ESE", "E  ", "ENE", "NE ", "NNE"};
char* winds[] = {"Calm", "Very Light", "Light", "Gentle", "Moderate", "Fresh", "Strong", "Moderate gale", "Fresh gale", "Strong gale", "Storm", "Violent storm", "Hurricane"};
int count;//index pointer for wind dir array
int wind;//index pointer for wind force array
int gust;
int force;
int windspeed;
int pressure = 1013;
//
// Timer stuff
Timer t;

// **** ETHERNET SETTING ****
// Arduino Uno pins: 10 = CS, 11 = MOSI, 12 = MISO, 13 = SCK
// Ethernet MAC address - must be unique on your network - MAC Reads T4A001 in hex (unique in your network)
byte mac[] = { 0x54, 0x34, 0x41, 0x30, 0x35, 0x32 }; 
byte ip[] = { 192, 168, 1, 121 }; // ip in lan
byte gateway[] = { 192, 168, 1, 1 }; // internet access via router
byte subnet[] = { 255, 255, 255, 0 }; //subnet mask                                      
// For the rest we use DHCP (IP address and such)
EthernetClient client;

char server[] = "192.168.1.117"; // IP Address (or name) of server to dump data to

unsigned long PreviousMillis = 0;// For when millis goes past app 49 days. 
unsigned long interval = 300000;  // Wait between dumps (5 min)
unsigned long intervalTime;      // Global var tracking Interval Time

volatile unsigned int rcounter = 0;      // Pulse counter for rain sensor - volatile for ISR
volatile unsigned long lastRainInterrupt = 0;   // Used for debounce timer - volatile for ISR
volatile unsigned long thisRainInterrupt = 0;   // Used for debounce timer - volatile for ISR
unsigned long raininterval = 3600000;  // used for rain updates every one hour
float rain;

// **** TEMPERATURE SETTINGS ****
// Sensor(s) data pin is connected to Arduino pin 6 in non-parasite mode!
OneWire  ds(6);

Adafruit_BMP085 bmp;

void setup() {
  #ifdef DEBUG
    Serial.begin(9600); // only use serial when debugging
    Serial.println("Setup");
  #endif
   pinMode(4,OUTPUT);
  digitalWrite(4,HIGH);
  Ethernet.begin(mac, ip);
  delay(1000);
  pinMode(2, INPUT);          // set pin 2 for rain sensor
 // pinMode(13, OUTPUT);
  attachInterrupt(0, addcount, FALLING);  // rain sensor on pin 2
  intervalTime = millis();  // Set initial target trigger time (right NOW!)
   t.every(raininterval, RainUpdateTimer); 
//
 if (!bmp.begin()) 
  {
  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  while (1) {}
  }
//
 Wire.begin(I2C_SLAVE_ADDRESS);        // join i2c bus for receiving from wind sensor
//  Wire.begin();
  //start the library, pass in the data details and the name of the serial port. 
  //Can be Serial, Serial1, Serial2, etc. 
  ET.begin(details(mydata), &Wire);
  //define handler function on receiving data
  Wire.onReceive(receive);
  //
//
  #ifdef DEBUG
    Serial.println("Temperature Drone - v2.11");
    Serial.println("-=-=-=-=-=-=-=-=-=-=-=-=-\n");
    Serial.print("IP Address        : ");
    Serial.println(Ethernet.localIP());
    Serial.print("Subnet Mask       : ");
    Serial.println(Ethernet.subnetMask());
    Serial.print("Default Gateway IP: ");
    Serial.println(Ethernet.gatewayIP());
    Serial.print("DNS Server IP     : ");
    Serial.println(Ethernet.dnsServerIP());
  #endif
} 
// -------------------
// Beginning of LOOP
// --------------------
void loop() {
//
 //digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
   EasyReceive(); // ET routine - get variable from I2C sensors 
   WindDirUpdate(); // Wind Direction Data - send to database
 //EasyReceive(); // ET routine - get variable from I2C sensors 
   WindSpeedUpdate(); // Wind Speed Data - send to database
   WindGustUpdate(); // Wind Gust data -send to database
   WindForceUpdate(); // Wind Force data - send to database
  // AirPressure(); // do Air pressure too
  //digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  delay(1500);              // wait for a second
//  
  t.update(); 
  unsigned long CurrentMillis = millis();
  
  if ( CurrentMillis < PreviousMillis ) // millis reset to zero?
  {
    intervalTime = CurrentMillis+interval;
  }
  
  if ( CurrentMillis > intervalTime )  // Did we reach the target time yet?
  {
    intervalTime = CurrentMillis + interval;
    
    if (!client.connect(server, 80)) {
    } else {
      #ifdef DEBUG
        Serial.println("Not Connected"); 
      #endif
      client.stop();
    } 
    
    // if you get a connected
    if (client.connect(server, 80)) 
    {
      #ifdef DEBUG
        Serial.println("-> Connected");  // only use serial when debugging
      #endif
      
      // Make a HTTP request:
      client.print( "GET /add_data2.php?");
      TemperaturesToGetVariables(); // send serial and temperature readings
      client.println( " HTTP/1.1");
      client.println( "Host: 192.168.1.117" );
      client.print(" Host: ");
      client.println(server);
      client.println( "Connection: close" );
      client.println();
      client.println();
      client.stop();
    }
    else 
    {
      // you didn't get a connection to the server:
      #ifdef DEBUG
        Serial.println("--> connection failed !?!?");  // only use serial when debugging
      #endif
    }
        AirPressure(); // do Air pressure too
  }
  else
  {
    Ethernet.maintain();
  }
  
   rain = (rcounter * .5); 
  
}
// end of VOID LOOP


//
void TemperaturesToGetVariables(void) 
{
  byte counter;
  byte present = 0;
  byte sensor_type;
  byte data[12];
  byte addr[8];
  float celsius;
  byte sensorcounter;
  
  ds.reset_search();
  sensorcounter = 1; // we start counting with sensor number 1
  
  while ( ds.search(addr) ) 
  {
    if (sensorcounter>1) client.print("&"); // add ampersand if not first sensor
    
    client.print("serial"); // print: sensorx=
    client.print(sensorcounter);
    client.print("=");
    
    #ifdef DEBUG
      // Print Serial number
      Serial.print("   Sensor     : ");
      Serial.println(sensorcounter);
      Serial.print("   Serial     : ");
    #endif
    
    for( counter = 0; counter < 8; counter++) 
    {
      if (addr[counter]<10) client.print("0");
      client.print(String(addr[counter], HEX));
      if (counter<7) client.print("%20");
      
      #ifdef DEBUG 
        if (addr[counter]<10) Serial.print("0");
        Serial.print(String(addr[counter], HEX));
        if (counter<7) Serial.print(" ");
      #endif
    }
    
    #ifdef DEBUG
      Serial.println();   // only use serial when debugging
    #endif
    
    client.print("&temperature");  // print: &temperaturex=
    client.print(sensorcounter);
    client.print("=");
    
    // Check CRC
    if (OneWire::crc8(addr, 7) != addr[7]) // print ERROR if CRC error
    {
        client.println("ERROR");
    }
    else // CRC is OK
    {    
        // Removed sensor type detection and assumed DS18B20 sensor
        ds.reset();
        ds.select(addr);
        ds.write(0x44);  // start conversion, with regular (non-parasite!) power
        
        delay(750);     // maybe 750ms is enough, maybe not
        
        present = ds.reset();
        ds.select(addr);    
        ds.write(0xBE);  // Read Scratchpad
        
        // Get Raw Temp Data
        for ( counter = 0; counter < 9; counter++) 
        {           // we need 9 bytes
          data[counter] = ds.read();
        }
        
        // Convert the data to actual temperature
        int16_t raw = (data[1] << 8) | data[0];
    
        // at lower res, the low bits are undefined, so let's zero them
        byte cfg = (data[4] & 0x60);
        
        //// default is 12 bit resolution, 750 ms conversion time
        if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
        else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
        else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    
    
        celsius = (float)raw / 16.0;
        client.print(celsius);
    
        #ifdef DEBUG 
          Serial.print("   Temperature: ");
          Serial.print(celsius);
          Serial.println(" C");
        #endif
     } 
     
     sensorcounter++;
  }
  return;
}

//----------------------------------------------
// Rain Routines 
//----------------------------------------------
// increment counter
void addcount(){
    // Make sure that we haven't updated the rain value in the last 100 milliseconds
  thisRainInterrupt = millis();
  if(thisRainInterrupt - lastRainInterrupt > 200){
     rcounter++;
     lastRainInterrupt = thisRainInterrupt;
  }
}
// --------------------
// Rain Update Routine
// --------------------
void RainUpdateTimer(){
    rain = (rcounter * .5);
 
  if (client.connect(server, 80)) {
    #ifdef DEBUG
    Serial.println("-> Connected for rain dump");
    Serial.println("Rain : ");
    Serial.println(rain);
    #endif
    // Make a HTTP request:
    client.print( "GET /add_datarain.php?");
    client.print("serial=");
    client.print( "raingauge" );
    client.print("&&");
    client.print("rain=");
    client.print(rain);
    client.println( " HTTP/1.1");
    client.print( "Host: " );
    client.println(server);
    client.println( "Connection: close" );
    client.println();
    client.println();
    client.stop();
    rcounter = 0; // reset the rain counter
  }
  else {
    // you didn't get a connection to the server:
    #ifdef DEBUG
    Serial.println("--> connection failed");
    #endif
    Ethernet.begin(mac, ip);
    Ethernet.maintain ();
  }
}
//-------------------------------------------------------------------------------
// Wind Direction Update Routine
//-------------------------------------------------------------------------------

void WindDirUpdate(){

  if (client.connect(server, 80)) {
    #ifdef DEBUG
    Serial.println("-> Connected for wind direction update");
    Serial.println("Wind Direction : ");
    Serial.println(count);
    #endif
    // Make a HTTP request:
    client.print( "GET /add_data_wind.php?");
    client.print("dir=");
    client.print(count);
    client.println( " HTTP/1.1");
    client.print( "Host: " );
    client.println(server);
    client.println( "Connection: close" );
    client.println();
    client.println();
    client.stop();
  }
  else {
    // you didn't get a connection to the server:
    #ifdef DEBUG
    Serial.println("--> connection to update wind direction failed");
    #endif
     Ethernet.begin(mac, ip);
    Ethernet.maintain ();
  }
}
//
//-------------------------------------------------------------------------------
// Wind Speed Update Routine
//-------------------------------------------------------------------------------

void WindSpeedUpdate(){
   
 
  if (client.connect(server, 80)) {
    #ifdef DEBUG
    Serial.println("-> Connected for wind speed update");
    Serial.println("Wind Speed : "); 
    Serial.println(windspeed);
    #endif
    // Make a HTTP request:
    client.print( "GET /add_data_wspeed.php?");
    client.print("speed=");
    client.print(windspeed);
    client.println( " HTTP/1.1");
    client.print( "Host: " );
    client.println(server);
    client.println( "Connection: close" );
    client.println();
    client.println();
    client.stop();
  }
  else {
    // you didn't get a connection to the server:
    #ifdef DEBUG
    Serial.println("--> connection to update wind speed failed/n");
    #endif
  }
}
//
// ----------------------------------------------------------
// Wind Gust Update
// ----------------------------------------------------------

void WindGustUpdate(){
 
  if (client.connect(server, 80)) {
    #ifdef DEBUG
    Serial.println("-> Connected for wind gust update");
    Serial.println("Wind Gust : ");
    Serial.println(gust);
    #endif
    // Make a HTTP request:
    client.print( "GET /add_data_wgust.php?");
    client.print("gust=");
    client.print(gust);
    client.println( " HTTP/1.1");
    client.print( "Host: " );
    client.println(server);
    client.println( "Connection: close" );
    client.println();
    client.println();
    client.stop();
  }
  else {
    // you didn't get a connection to the server:
    #ifdef DEBUG
    Serial.println("--> connection to update wind gust failed/n");
    #endif
  }
}
//
// ----------------------------------------------------------
// Wind Force Update
// ----------------------------------------------------------

void WindForceUpdate(){
 
  if (client.connect(server, 80)) {
    #ifdef DEBUG
    Serial.println("-> Connected for wind force update");
    Serial.println("Wind Force : ");
    Serial.println(force);
    #endif
    // Make a HTTP request:
    client.print( "GET /add_data_wforce.php?");
    client.print("force=");
    client.print(force);
    client.println( " HTTP/1.1");
    client.print( "Host: " );
    client.println(server);
    client.println( "Connection: close" );
    client.println();
    client.println();
    client.stop();
  }
  else {
    // you didn't get a connection to the server:
    #ifdef DEBUG
    Serial.println("--> connection to update wind force failed/n");
    #endif
  }
}
//
//----------------------------------------------------------
//Easy Transfer routines
//----------------------------------------------------------
//check and see if a data packet has come in.
//
void EasyReceive(){
  if(ET.receiveData()){
  //access the variables from the data struct. [name of the group].[variable name]  
  //
  //
  if (mydata.windcount != -1) {
    #ifdef DEBUG
    Serial.print("Direction: ");
    Serial.println(mydata.windcount);         // debug print the wind count integer
    #endif
    count = mydata.windcount; // get counter from wind direction sensor
  }
  if (mydata.etgust != -1) {
    #ifdef DEBUG
    Serial.print("Gust: ");
    Serial.println(mydata.etgust);         // debug print the wind gust integer
    #endif
    gust = mydata.etgust; // get gust 
  }
  if (mydata.etwindspeed != -1) {
    #ifdef DEBUG
    Serial.print("Speed: ");
    Serial.println(mydata.etwindspeed);         // debug print the wind speed integer
    Serial.println("");
    #endif
    windspeed = mydata.etwindspeed; // get windspeed
  }
  if (mydata.etwindforce != -1) {
    #ifdef DEBUG
    Serial.print("Force: ");
    Serial.println(mydata.etwindforce);         // debug print the wind force integer
    Serial.println("");
    #endif
    force = mydata.etwindforce; // get windforce
  }
  wind = count / 5;//divide count by 5 for 16 points for Direction array chars
  }
}
//
//-------------------------------
//I2C receive routine 
//-------------------------------
void receive(int numBytes) {}
//

//-------------------
// Air Pressure
//-------------------
void AirPressure()
{
   pressure = (bmp.readPressure() / 100);
   if ( pressure > 1080 )  { // check if reading is a valid value
   pressure = (bmp.readPressure() / 100); // if not, read again
  }
  if (client.connect(server, 80)) {
    #ifdef DEBUG
    Serial.println("-> Connected for air pressure update");
    Serial.println("Pressure : ");
    Serial.println(pressure);
    #endif

   // Make a HTTP request:
    client.print( "GET /add_data_press.php?");
    client.print("serial=");
    client.print( "baro" );
    client.print("&&");
    client.print("pressure=");
    client.print(pressure);
    client.println( " HTTP/1.1");
    client.print( "Host: " );
    client.println(server);
    client.println( "Connection: close" );
    client.println();
    client.println();
    client.stop();
  }
  else {
    // you didn't get a connection to the server:
   #ifdef DEBUG
    Serial.println("--> connection to update air pressure failed/n");
   #endif
  }
}

