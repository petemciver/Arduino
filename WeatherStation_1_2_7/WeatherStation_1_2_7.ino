/*--------------------------------------------------------------
  Program:      eth_websrv_temp_humid_pressure_winddir_windspeed_sensors

  Description:  Arduino web server weather station 
                shows the state of sensors on a web page. 
                Does not use the SD card... (yet)
  
  Hardware:     Arduino Mega 2560 (Uno did not have enough RAM for global variables)
                Ethernet shield.
                Dallas Semiconductor DS18B20 1-wire temp sensor
                DHT22 Temp & Humidity sensor
                Works with the Adafruit BMP085 baro breakout board 
                Rotary encoder for wind direction with dedicated Arduino Pro-Mini
                Anemometer, wind speed sensor with dedicated Arduino Pro-Mini
                rain gauge routines
                later... light level.
                
  Software:     ver 1.2.7
                Developed using Arduino 1.6.4 software
  
  References:   - WebServer example by David A. Mellis 
                - Modified for additional sensors by Peter McIver
                - Ethernet library documentation:
                  http://arduino.cc/en/Reference/Ethernet
                - https://www.adafruit.com/products/391

  Date:         19 September 2015
--------------------------------------------------------------*/
// BMP085
// Connect VCC of the BMP085 sensor to 3.3V (NOT 5.0V!)
// Connect GND to Ground
// Connect SCL to i2c clock - on Arduino Uno thats Analog 5
// Connect SDA to i2c data - on Arduino Uno thats Analog 4
// EOC is not used, it signifies an end of conversion
// XCLR is a reset pin, also not used here

#include <Wire.h>
#include <EasyTransferI2C.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <DHT.h>
#include <Adafruit_BMP085.h>
#include <Ethernet.h>
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
//
//Uncomment one of the following depending on the orientation of the encoder
//1. Shaft pointing up array
//char* Dirs[]={"N  ", "NNE", "NE ", "ENE", "E  ", "ESE", "SE ", "SSE", "S  ", "SSW", "SW ", "WSW", "W  ", "WNW", "NW ", "NNW"};
//2. Shaft pointing down array
char* Dirs[]={"N  ", "NNW", "NW ", "WNW", "W  ", "WSW", "SW ", "SSW", "S  ", "SSE", "SE ", "ESE", "E  ", "ENE", "NE ", "NNE"};
char* winds[] = {"Calm", "Light air", "Light breeze", "Gentle breeze", "Moderate breeze", "Fresh breeze", "Strong breeze", "Moderate gale", "Fresh gale", "Strong gale", "Storm", "Violent storm", "Hurricane"};
int count;//index pointer for wind dir array
int wind;//index pointer for wind force array
int gust;
int force;
int windspeed;
int pressure;
int p1;
int p2;
int p3;
char* btrend = "stable";
char state;
volatile unsigned int rcounter = 0;      // Pulse counter for rain sensor 
volatile unsigned long lastRainInterrupt = 0;   // Used for debounce timer
volatile unsigned long thisRainInterrupt = 0;   // Used for debounce timer
int rain;
int rainacc24;
static unsigned long nextSwitchTime = millis()+600000L;//set timer to 10 min intervals
//
#define DHTPIN 7 // DHT sensor connected to pin 7
#define DHTTYPE DHT22 // sensor type
DHT dht(DHTPIN, DHTTYPE);
float humid;
float temp;
int fan = 8; //fan relay pin
//
#define REF_PIN 6 // 1-wire temp sensor on pin 6
void getCurrentTemp( int *sign, int *whole, int *fract);
char temp_string[10];
//
// MAC address from Ethernet shield sticker under board
byte mac[] = { 0xAA, 0xBA, 0xBE, 0xEF, 0xFE, 0xE0 };
IPAddress ip(192, 168, 1, 118); // IP address, may need to change depending on network
EthernetServer server(80);  // create a server at port 80

Adafruit_BMP085 bmp;

void setup()
{
    Ethernet.begin(mac, ip);    // initialize Ethernet device
    server.begin();             // start to listen for clients
    pinMode(fan, OUTPUT);       // input pin for fan relay
    digitalWrite(fan, LOW);     // turn fan off
     // initialize DS18B20 datapin
    digitalWrite(REF_PIN, LOW);
    pinMode(REF_PIN, INPUT);    // sets the digital pin as input (logic 1)
    pinMode(15, INPUT);         // find out what this does - pin A1..?
    pinMode(2, INPUT);          // set pin 2 for rain sensor
    attachInterrupt(0, addcount, FALLING);  // rain sensor on pin 2
    Serial.begin(9600);        // start serial for output
  if (!bmp.begin()) 
  {
	Serial.println("Could not find a valid BMP085 sensor, check wiring!");
	while (1) {}
  }
 Wire.begin(I2C_SLAVE_ADDRESS);        // join i2c bus for receiving from wind sensor
  //start the library, pass in the data details and the name of the serial port. 
  //Can be Serial, Serial1, Serial2, etc. 
  ET.begin(details(mydata), &Wire);
  //define handler function on receiving data
  Wire.onReceive(receive);
}

void loop()
{
    EthernetClient client = server.available();  // try to get client

    if (client) {  // got client?
        boolean currentLineIsBlank = true;
        while (client.connected()) {
            if (client.available()) {   // client data available to read
                char c = client.read(); // read 1 byte (character) from client
                // last line of client request is blank and ends with \n
                // respond to client only after last line received
                if (c == '\n' && currentLineIsBlank) {
                    // send a standard http response header
                    client.println("HTTP/1.1 200 OK");
                    client.println("Content-Type: text/html");
                    client.println("Connnection: close");
                    client.println();
                    // send web page
                    client.println("<!DOCTYPE html>");
                    client.println("<html>");
                    client.println("<head>");
                    client.println("<title>Pete's Arduino Weather Station</title>");
                    client.println("<meta http-equiv=\"refresh\" content=\"5\">");
                    client.println("</head>");
                    client.println("<body>");
                    client.println("<h1>Pete's Weather Station Project MkVII</h1>");
  //                  client.println("<p>The DHT22 Garage Sensor (D7) is reading:</p>");
                    GetTempHumidity(client);
                    PrintWebData(client);
                    client.println("</body>");
                    client.println("<footer><br><br><p>Arduino-powered, Weather Station.<br>Arduino 1: Arduino Mega2560 with Ethernet shield<br>Arduino 2: Pro-Mini (wind direction)<br>Arduino 3: Pro-Mini (wind speed)<br>Sensors: DHT22, DS18B20, BMP085, Rotary Encoder<br>Comms Protocols: I2C, 1-wire<br>P.D. McIver September 2015<br>Next stage: Rain Gauge.</p></footer>");
                    client.println("</html>");
                    break;
                }
                // every line of text received from the client ends with \r\n
                if (c == '\n') {
                    // last character on line of received text
                    // starting new line with next character read
                    currentLineIsBlank = true;
                } 
                else if (c != '\r') {
                    // a text character was received from client
                    currentLineIsBlank = false;
                }
            } // end if (client.available())
        } // end while (client.connected())
        delay(2);      // give the web browser time to receive the data
        client.stop(); // close the connection
    } // end if (client)
//-------------------------------------------------------
//Optional environment control routine
//Check temp in case no clients and turn fan on or off (needs work)
// float h = dht.readHumidity();
//  float t = dht.readTemperature();
//  if (isnan(t) || isnan(h)) {
//      digitalWrite(fan, LOW); // turn fan off   
//  } 
//  else {
//    if (t >= 37) {
//      digitalWrite(fan, HIGH); // turn fan on
//    }
//    else {
//      digitalWrite(fan, LOW); // turn fan off
//   }
// }
//----------------------------------------------------------
//
//Easy Transfer routines
//check and see if a data packet has come in.
//
  if(ET.receiveData()){
  //access the variables from the data struct. [name of the group].[variable name]  
  //
  //
  if (mydata.windcount != -1) {
    Serial.print("Direction: ");
    Serial.println(mydata.windcount);         // debug print the wind count integer
    count = mydata.windcount; // get counter from wind direction sensor
  }
  if (mydata.etgust != -1) {
    Serial.print("Gust: ");
    Serial.println(mydata.etgust);         // debug print the wind gust integer
  gust = mydata.etgust; // get gust 
  }
  if (mydata.etwindspeed != -1) {
    Serial.print("Speed: ");
    Serial.println(mydata.etwindspeed);         // debug print the wind speed integer
  windspeed = mydata.etwindspeed; // get windspeed
  }
  if (mydata.etwindforce != -1) {
  force = mydata.etwindforce; // get windspeed
  }
  wind = count / 5;//divide count by 5 for 16 points
  }
  //Trending Timer

  if( nextSwitchTime < millis() )
    {
        nextSwitchTime = millis() + 600000L;
        BaroTrend();  // go to trending routine
    }
  
  //
} // end of void loop
// Temperature & Humidity routing for DHT sensor
void GetTempHumidity(EthernetClient cl)
{
//
// Reading temperature or humidity takes about 250 milliseconds!
// Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
// delay(50);//wait a bit
float h = dht.readHumidity();
humid = h;
float t = dht.readTemperature();
temp = t;
// check if returns are valid, if they are NaN (not a number) then something went wrong!
if (isnan(t) || isnan(h)) {
  //cl.println("DHT read fail... Please wait...");
  Serial.print("DHT read Fail ... Houston we have a problem");
  }
} 

void PrintWebData(EthernetClient cl)
{
  cl.println("<p>Inside Garage Sensor:</p>");
  cl.println("<p>Relative Humidity: ");
  cl.println(humid);
  cl.println("%\t</p>");
  cl.println("<p>Inside Temperature: ");
  cl.println(temp);
  cl.println("&#176;C    </p>");
//  if (t >= 37) {
//      digitalWrite(fan, HIGH); // turn fan on
//      cl.println("<p>Ventilation fan is running</p>");
//    }
//    else {
//      digitalWrite(fan, LOW); // turn fan off
//      cl.println("<p>Ventilation fan is not running (disconnected)</p>");
//    }
    // read 1-wire temp sensor here...
    // cl.println("<p>The DS18B20 external sensor on (D6) is reading:</p>");
    getCurrentTemp(temp_string);
    cl.println("<p>Outside Temperature: ");
    cl.println(temp_string);
    cl.println(" &#176;C");
    cl.println("</p>");
    cl.println("<p>Air Pressure : ");
    pressure = (bmp.readPressure() / 100);
    cl.println(pressure);
    cl.println(" hPa");
    cl.println(btrend);
    cl.println("</p>");
    cl.println("<p>Wind : ");
    //cl.println(force);
    cl.println("&#20; ");
    cl.println(winds[force]);
    cl.println("</p>");
    cl.println("<p>");
    cl.println(Dirs[wind]);      // print the wind direction from the array
    cl.println("&#20;&#20; ");
    cl.println(windspeed);      // print the wind speed
    cl.println(" km/h");
    cl.println("</p>");  
    cl.println("<p>Gust : ");
    cl.println(gust);      // print the wind gust
    cl.println(" km/h in last 10min.");
    cl.println("<p>Rain: ");
    rain = (rcounter * .25);  // initial rain calc - need counters & reset routines
    cl.println(rain);      // print the rain falling in the last hour
    cl.println(" mm/hr");
    cl.println("Counter : ");
    cl.println(rcounter);
    cl.println("</p>");  
    cl.println("<p>Accumulated rainfall : ");
    rainacc24 = (rcounter * .25);  
    cl.println(rainacc24);      // print the accumulated rainfall over 24hrs
    cl.println(" mm/hr in last 24hr."); 
    cl.println("</p>");
    //
}
// 1-wire routines:
// RESET
void OneWireReset (int Pin) // reset.  Should improve to act as a presence pulse
{
  digitalWrite (Pin, LOW);
  pinMode (Pin, OUTPUT);        // bring low for 500 us
  delayMicroseconds (500);
  pinMode (Pin, INPUT);
  delayMicroseconds (500);
}

// OUTPUT
void OneWireOutByte (int Pin, byte d) // output byte d (least sig bit first).
{
  byte n;

  for (n=8; n!=0; n--)
  {
    if ((d & 0x01) == 1)  // test least sig bit
    {
      digitalWrite (Pin, LOW);
      pinMode (Pin, OUTPUT);
      delayMicroseconds (5);
      pinMode (Pin, INPUT);
      delayMicroseconds (60);
    }
    else
    {
      digitalWrite (Pin, LOW);
      pinMode (Pin, OUTPUT);
      delayMicroseconds (60);
      pinMode (Pin, INPUT);
    }

    d = d>>1; // now the next bit is in the least sig bit position.
  }
}

// INPUT
byte OneWireInByte (int Pin) // read byte, least sig byte first
{
  byte d, n, b;

  for (n=0; n<8; n++)
  {
    digitalWrite (Pin, LOW);
    pinMode (Pin, OUTPUT);
    delayMicroseconds (5);
    pinMode (Pin, INPUT);
    delayMicroseconds (5);
    b = digitalRead (Pin);
    delayMicroseconds (50);
    d = (d >> 1) | (b<<7); // shift d to right and insert b in most sig bit position
  }
  return (d);
}

// GET TEMPERATURE
void getCurrentTemp (char *temp)
{
  int HighByte, LowByte, TReading, Tc_100, sign, whole, fract;

  OneWireReset (REF_PIN);
  OneWireOutByte (REF_PIN, 0xcc);
  OneWireOutByte (REF_PIN, 0x44); // perform temperature conversion, strong pullup for one sec

  OneWireReset (REF_PIN);
  OneWireOutByte (REF_PIN, 0xcc);
  OneWireOutByte (REF_PIN, 0xbe);

  LowByte = OneWireInByte (REF_PIN);
  HighByte = OneWireInByte (REF_PIN);
  TReading = (HighByte << 8) + LowByte;
  sign = TReading & 0x8000;  // test most sig bit
  if (sign) // negative
  {
    TReading = (TReading ^ 0xffff) + 1; // 2's comp
  }
  Tc_100 = (6 * TReading) + TReading / 4;    // multiply by (100 * 0.0625) or 6.25

  whole = Tc_100 / 100;  // separate off the whole and fractional portions
  fract = Tc_100 % 100;

  if (sign) {
    temp[0] = '-';
  } else {
    temp[0] = '+';
  }

  if (whole/100 == 0) {
    temp[1] = ' ';
  } else {
    temp[1] = whole/100+'0';
  }

  temp[2] = (whole-(whole/100)*100)/10 +'0' ;
  temp[3] = whole-(whole/10)*10 +'0';
  temp[4] = '.';
  temp[5] = fract/10 +'0';
  temp[6] = fract-(fract/10)*10 +'0';
  temp[7] = '\0';
}

//I2C receive routine -------------------------------
void receive(int numBytes) {}

//Air Pressure trending routine --------------------------
void BaroTrend()
{
  if ( p3 > p1 )
  { 
    btrend = (" rising"); 
}
    if ( p3 < p1 )
  { 
    btrend = (" rising"); 
}
  else
  { 
    btrend = (" stable"); 
}
  p3 = p2;
  p2 = p1;
  p1 = pressure;
}

// Rain Routines ----------------------------------------------
// increment counter
void addcount(){
    // Make sure that we haven't updated the rain value in the last 100 milliseconds
  thisRainInterrupt = millis();
  if(thisRainInterrupt - lastRainInterrupt > 100){
     rcounter++;
     lastRainInterrupt = thisRainInterrupt;
  }
}


//

