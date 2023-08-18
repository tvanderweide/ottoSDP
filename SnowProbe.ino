  /******************************************************************************
  Log GPS data and Lidar Distance to a CSV file when the button is pushed
  By Thomas Van Der Weide
  March 16, 2023

  Use the NeoGPS library to parse the NMEA strings sent by the GPS module,
  and print interesting GPS information - comma separated - to a newly created
  file on the SD card.

  Resources:
  SD Library (Built-in)
  NeoGPS Library (NMEAGPS)
  Adafruit_NeoPixel
  LiquidCrystal_I2C
  LIDARLite_v4LED.h

  Development/hardware environment specifics:
  Arduino Feather M0
  Emlid Reach M2
  - Update GPS Firmware
  - Set Position Output to Serial -> UART with 115200 baud
    Emlid S1 Ground  (pin 1) to Arduino GND
    Emlid S1 TX (pin 4) to Arduino RX
  Garmin LidarLite LED v4
    LIDAR-Lite 5 VDC   (pin 1) to Arduino 5V
    LIDAR-Lite Ground  (pin 2) to Arduino GND
    LIDAR-Lite I2C SDA (pin 3) to Arduino SDA
    LIDAR-Lite I2C SCL (pin 4) to Arduino SCL
    (Capacitor recommended to mitigate inrush current when device is enabled)
    680uF capacitor (+) to Arduino 5V
    680uF capacitor (-) to Arduino GND]
  Trigger Button
    Pin 11
  LCD Screen
    SDA/SCL/3.3V/GND
******************************************************************************/

  #include <SPI.h>
  #include <SD.h>
  #include <NMEAGPS.h>
  #include <Adafruit_NeoPixel.h>
  #include <Wire.h> // Library for I2C communication
  #include <LiquidCrystal_I2C.h> // Library for LCD
  #include <RunningMedian.h>

  #include <stdint.h>
  #include "LIDARLite_v4LED.h"
  //#include <LIDARLite_v4LED.h>

///////////////////////////////
//// Lidar Config  ////// ////
/////////////////////////////
LIDARLite_v4LED myLidarLite;
#define FAST_I2C
//#define MonitorPin    10  //This is wrong (6)
//#define TriggerPin    11 //This might also be wrong (5)

uint16_t distance;
uint16_t  newDistance;

enum rangeType_T
{
    RANGE_NONE,
    RANGE_SINGLE,
    RANGE_CONTINUOUS,
    RANGE_SINGLE_GPIO,
    RANGE_CONTINUOUS_GPIO
};

///////////////////////////////
////////  GPS config  ////////
/////////////////////////////
// Check that the GPS config files are set up properly
#if !defined(NMEAGPS_PARSE_RMC) & \
    !defined(NMEAGPS_PARSE_GGA) & \
    !defined(NMEAGPS_PARSE_GLL)
#error You must uncomment at least one of NMEAGPS_PARSE_RMC, NMEAGPS_PARSE_GGA or NMEAGPS_PARSE_GLL in NMEAGPS_cfg.h!
#endif

#if !defined(GPS_FIX_LOCATION)
#error You must uncomment GPS_FIX_LOCATION in GPSfix_cfg.h!
#endif

using namespace NeoGPS;
#define gpsPort Serial1
#define GPS_BAUD 115200 // GPS module's baud rate
static NMEAGPS gps;// This parses the GPS characters
static gps_fix nextFix;// This holds the latest GPS fix
static gps_fix startLocation; // starting GPS data
static gps_fix currentFix; // starting GPS data
NeoGPS::time_t  startGPSTime = 0;
uint16_t startGPSTimeMS = 0;

///////////////////////////////
//// Variable Definitions ////
/////////////////////////////
#define DEBUG_PORT SERIAL_PORT_USBVIRTUAL
#define BAUD_USB_SERIAL 9600

//File to log to
File logFile;
// SD card read/write
#define CS 4
//NeoPixel light strip
int ledPin = 6;
// Trigger for logging new location and distance
int buttonPin = 11;
// Interrupt flags thrown but button press
volatile bool myInt2 = false;

int firstDistLog = 0;
int firstColLog = 0;
int count = 0;//Keeps track of number of logs recorded
int calibration = 0;//Record calibration distance reading and use for snow depth calculation and display
unsigned long btnPress = 0; //used to keep track of when trigger button was pressed
unsigned long waitTime = 5000; // time to attempt to get a new GPS reading

// Logging variables
char filename[15];
#define LOG_COLUMN_COUNT 11
char* log_col_names[LOG_COLUMN_COUNT] = {
  "Count", "Distance", "latitude", "longitude", "altitude", "sat_count", "HDOP", "dateTime", "GPSTime", "status"
};
/*
   Lighting Variables
   NeoPixel strip light meaning
   Light 8 - SD card available
   Light 7 - LogFile can be written to
   Light 6 - Distance Calibration recorded
   Light 5 - GPS Fix
   Light 4 - Button Press
   Light 3 - Distance > 30cm successfully measured?
   Light 2 - Valid GPS coordinates received?
   Light 1 - Was the location recorded?
*/
int LED0 = 8;
int LED1 = 7;
int LED2 = 6;
int LED3 = 5;
int LED4 = 4;
int LED5 = 3;
int LED6 = 2;
int LED7 = 1;
int LED8 = 0;
#define NUM_LEDS 8
#define lightInterval 1000 // How often to update the lights
unsigned long periodBlue = 5000; // Max time between GPS updated readings (for light display)
unsigned long periodRed = 20000; // Max time between GPS updated readings (for light display)
unsigned long updateLightTime = 0; //used to keep track of how often the lights have been updated
unsigned long updateGPSTime = 0; //used to keep track of how often a valid GPS reading is received
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, ledPin, NEO_GRB + NEO_KHZ800);
uint32_t red = strip.Color(255, 0, 0);
uint32_t orange = strip.Color(255, 128, 0);
uint32_t yellow = strip.Color(255, 255, 0);
uint32_t green = strip.Color(0, 255, 0);
uint32_t light_blue = strip.Color(0, 255, 255);
uint32_t blue = strip.Color(0, 0, 255);
uint32_t off = strip.Color(0, 0, 0);


// Screen
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);
// for LCD screen update
long previousLCDMillis = 0;
long lcdInterval = 2000;
int calibFlag = 0; //Keep LCD screen on calibration
int distFlag = 1; //Swap between LCD screens

///////////////////////////////
//////  Setup Function   /////
/////////////////////////////
void setup()
{
  // Initiate the LCD:
  lcd.init();
  lcd.backlight();
  // Print the first line of the LCD:
  lcd.setCursor(0, 0); // Set the cursor on the first column and first row.
  lcd.print("Snow Depth Probe"); // Print the string "Hello World!"
  lcd.setCursor(1, 1); //Set the cursor on the second column and the second row (counting starts at 0!).
  lcd.print("Initializing...");

  pinMode(buttonPin, INPUT_PULLDOWN); // button
  attachInterrupt(digitalPinToInterrupt(buttonPin), detectInt2, RISING);
  pinMode(CS, OUTPUT); //necessary for the SD card
  gpsPort.begin(GPS_BAUD);
  strip.begin();

  DEBUG_PORT.begin(BAUD_USB_SERIAL);
  DEBUG_PORT.println("Debug Port Started..");
  DEBUG_PORT.println("Setting up SD card.");
  if (!SD.begin(CS)) {
    DEBUG_PORT.println("Card init. failed!");
    perror("2");
    strip.setPixelColor(LED0, red); // Set first light to Red to indicate bad SD card
  }
  else {
    strip.setPixelColor(LED0, blue); // Set first light to blue to indicate SD card is ready
  }

  strcpy(filename, "LOG00.TXT");
  for (uint8_t i = 0; i < 100; i++) {
    filename[3] = '0' + i / 10;
    filename[4] = '0' + i % 10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }
  DEBUG_PORT.println(filename);
  
  // Open the file and print "Calibration: " text at the top
  printTest();

  
  // Lidar Setup
  // Initialize Arduino I2C for communication to Lidar
  Wire.begin();
  #ifdef FAST_I2C
      #if ARDUINO >= 157
          Wire.setClock(400000UL); // Set I2C frequency to 400kHz (for Arduino Due)
      #else
          TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
      #endif
  #endif

  // ---------------------------------------------------------------------------------------------------
  // External pull-ups are NOT present on the Arduino feather m0 and must be added manually. 
  // Needs 2.2k-10K pull-up resistors to 3.3V
  // ---------------------------------------------------------------------------------------------------
//  pinMode(MonitorPin, INPUT);// Optional GPIO pin assignments for measurement triggering & monitoring
//  pinMode(TriggerPin, OUTPUT);// Optional GPIO pin assignments for measurement triggering & monitoring
//  digitalWrite(TriggerPin, LOW);// Optional GPIO pin assignments for measurement triggering & monitoring7
  myLidarLite.configure(1); 

  smartDelay(5000);
  
}//End Setup

///////////////////////////////
//////  Loop Function   /////
/////////////////////////////
void loop()
{
  if (firstColLog == 0)
  {
    distCalib();
  }
  else if (myInt2)
  {
    doWork();
    myInt2 = false;
  }
  else
  {
    smartDelay(75);
    updateLights();
    updateScreen();
  }
} //end loop

// Detects if the trigger button has been pressed
void detectInt2() {
  myInt2 = true;
}

static void lcdCalib()
{
  // Update the LCD:
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Dist calibration"); //Remind user to calibrate
  lcd.setCursor(1, 1);
  lcd.print("     Needed     "); //Remind user to calibrate  
}

static void lcdSats()
{
  float hdopVal = nextFix.hdop / 1000.0;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(" Sats: ");
  lcd.print(nextFix.satellites); // Prints the # of Satellites
  if (millis() - updateGPSTime > periodRed)
  {
    lcd.print(" (OLD)");
  }
  lcd.setCursor(0, 1);
  lcd.print(" HDOP: ");
  lcd.print(hdopVal); // Prints the HDOP Value (not sure how accurate this is)
  if (millis() - updateGPSTime > periodRed)
  {
    lcd.print(" (OLD)");
  }
}

static void lcdDistance()
{
  float range = 0.0;
  if (firstDistLog == 0)
  {
    range = 0.0;
  }
  else
  {
    range = startLocation.location.DistanceKm( nextFix.location );
  }
  range = range * 1000.0;//Put into meters
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Distance: ");
  lcd.print(int(range)); // Prints the distance from calibration point
  lcd.print(" m");
  lcd.setCursor(0, 1);
  lcd.print("Count: ");
  lcd.print(count); // Prints number of depths recorded
  lcd.print("  (");
  lcd.print(filename[3]);//Prints logFileName
  lcd.print(filename[4]);//second digit
  lcd.print(")");
}

static void lcdDepth()
{
  float depth = calibration - distance;
  depth = depth / 1000.0;
  // Display the distance on the LCD:
  lcd.clear();
  lcd.setCursor(0, 0); // Set the cursor to column 1, line 1
  lcd.print(" Depth  at ");
  lcd.print(count);
  lcd.setCursor(5, 1);
  lcd.print(depth, 3); // Prints the measured snow depth
  lcd.print(" m"); // Prints "m" on the LCD
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (gps.available(gpsPort))
    {
      nextFix = gps.read(); // save the latest
      if (nextFix.valid.status && (nextFix.status >= gps_fix::STATUS_STD))
      {
        updateGPSTime = millis();
      }
    }
  } while (millis() - start < ms);
}

static void updateScreen()
{
  if (millis() > previousLCDMillis + lcdInterval)
  {
    if (distFlag == 0)
    {
      lcdDistance();
      distFlag = 1;
    }
    else
    {
      lcdSats();
      distFlag = 0;
    }
    previousLCDMillis = millis();
  }
}

static void updateLights() {
  if (millis() > updateLightTime + lightInterval) {
    // Update GPS light
    if (updateGPSTime == 0) {
      strip.setPixelColor(LED3, red);// Set fourth light to red to indicate never received GPS signal
    }
    else if (millis() - updateGPSTime <= periodBlue) {
      if ((currentFix.hdop / 1000.0) < 2.0) {
        strip.setPixelColor(LED3, blue);// Set fourth light to blue to indicate good GPS fix
      }
      else {
        strip.setPixelColor(LED3, light_blue);// Set fourth light to light_blue to indicate valid GPS fix
      }
    }
    else if (millis() - updateGPSTime > periodBlue) { //test whether the periodBlue has elapsed
      if (millis() - updateGPSTime > periodRed) {
        strip.setPixelColor(LED3, red);// Set fourth light to red to indicate no GPS signal
      }
      else {
        strip.setPixelColor(LED3, yellow);// Set fourth light to yellow to indicate poor GPS signal
      }
    }
    else {
      strip.setPixelColor(LED3, red);// Set fourth light to red to indicate no GPS connection
    }
    updateLightTime = millis();
    strip.show();
  }
}//End Update Light

bool gpsLoop(bool fresh_dataLoop, unsigned long waitTimeLoop)
{
  do
  {
    while (gps.available(gpsPort))
    {
      nextFix = gps.read(); // save the latest
      if (nextFix.valid.status  && (nextFix.status >= gps_fix::STATUS_STD) && nextFix.valid.location && nextFix.valid.date && nextFix.valid.satellites)
      {
        currentFix = nextFix;
        updateGPSTime = millis();
        updateLights();
        fresh_dataLoop = true;
        if(startGPSTime == 0)
        {
          startGPSTime = currentFix.dateTime;
          startGPSTimeMS = currentFix.dateTime_cs * 10;
        }
        return fresh_dataLoop;
      }
    }
  } while (millis() - btnPress <= waitTimeLoop);
  return fresh_dataLoop;
}//End gpsLoop

static void distCalib()
{
  if (calibFlag == 0)
  {
    lcdCalib();
    calibFlag = 1;
    strip.setPixelColor(LED2, orange);
    strip.show();
  }
  
  if (myInt2)
  {
    newDistance = distanceContinuous(&distance);
    if (newDistance == 1 && distance > 100 && distance < 4500)
    {
      DEBUG_PORT.println(distance);
      calibration = distance;
      printCalibration();
      myInt2 = false;
    }
  }
  else
  {
    smartDelay(75);
    updateLights();
  }
}

static void doWork()
{
  newDistance = 0;
  strip.setPixelColor(LED4, orange);// Set fifth light to orange to indicate button press
  strip.show();
  btnPress = millis();

  boolean fresh_data = false;
  startGPSTime = 0;
  startGPSTimeMS = 0;
  
  //Sensor stuff
  newDistance = distanceContinuous(&distance);
  if (newDistance && distance > 100 && distance < 4500) {
    DEBUG_PORT.println(distance);
    // Use a light to indicate if the distance reading is valid (greater than the minimum distance)
    strip.setPixelColor(LED5, green);// Set sixth light to green to indicate valid distance
    strip.show();
    lcdDepth();
    fresh_data = gpsLoop(fresh_data, 5000);
  }
  else { // Set lights to orange to indicate non-valid distance and exit without saving
    strip.setPixelColor(LED5, orange);
    strip.setPixelColor(LED6, orange);
    strip.setPixelColor(LED7, orange);
    strip.show();
    lcdDepth();
    smartDelay(1200);
    strip.setPixelColor(LED4, off);
    strip.setPixelColor(LED5, off);
    strip.setPixelColor(LED6, off);
    strip.setPixelColor(LED7, off);
    strip.show();
    lcdSats();
    return;
  }
  
  if (fresh_data)
  {
    strip.setPixelColor(LED6, green);// Set sevength light to green to indicate GPS coordinates are valid
    if (logGPSData())
    {
      strip.setPixelColor(LED4, green);
      strip.setPixelColor(LED7, green);
      DEBUG_PORT.print("Button Press time: ");
      DEBUG_PORT.println(btnPress);
      DEBUG_PORT.print("GPS time: ");
      DEBUG_PORT.println(updateGPSTime);
      DEBUG_PORT.println("GPS logged.");
    }
    else // If we failed to log GPS coordinates
    {
      strip.setPixelColor(LED4, red);// Set seventh light to red to indicate failure to log coordinates and distance
      strip.setPixelColor(LED7, red);
      DEBUG_PORT.println("Failed to log new GPS data.");
    }
  }
  else // If GPS data isn't valid
  {
    strip.setPixelColor(LED4, red);
    strip.setPixelColor(LED6, red);
    strip.setPixelColor(LED7, red);
    // Print a debug message. Maybe we don't have enough satellites yet.
    lcd.clear();
    lcd.setCursor(0, 0); // Set the cursor to column 1, line 1
    lcd.print("  No GPS data.  "); // Prints string "Display = " on the LCD
    lcd.setCursor(1, 1);
    lcd.print(" Sats: ");
    lcd.print(currentFix.satellites); // Prints the number of Sats
  }

  strip.show();
  smartDelay(1200);
  strip.setPixelColor(LED4, off);
  strip.setPixelColor(LED5, off);
  strip.setPixelColor(LED6, off);
  strip.setPixelColor(LED7, off);
  strip.show();
  lcdSats();
  return;
}//End doWork


byte logGPSData()
{
  logFile = SD.open(filename, FILE_WRITE);

  if (logFile)
  {
    // Print "Count", "Distance", "latitude", "longitude", "altitude", "sat_count", "HDOP", "dateTime", "Year/Month/Day Hour:Minute:Seconds.CS", "status"
    logFile.print(count);
    logFile.print(',');
    logFile.print(distance);
    logFile.print(',');
    // Print Latitude and Lon
    logFile.print(currentFix.latitude(), 7);
    logFile.print(',');
    logFile.print(currentFix.longitude(), 7);
    logFile.print(',');
    logFile.print(currentFix.alt.whole, 7);
    logFile.print(',');
    // Print # of satellites and HDOP
    logFile.print(currentFix.satellites);
    logFile.print(',');
    float hdopVal = currentFix.hdop / 1000.0;
    logFile.print(hdopVal);
    logFile.print(',');
    //Print DateTime in UTC in seconds after EPOCH
    logFile.print(currentFix.dateTime); // Raw date in seconds after epoch
    logFile.print(',');
    
    // startGPSTime
    // Year/Month/Day Hour:Minute:Seconds.CS
    logFile.print("20");
    logFile.print(startGPSTime.year);//year
    logFile.print('/');
    if (startGPSTime.month < 10)//month
    {
      logFile.print("0");
    }
    logFile.print(startGPSTime.month);
    logFile.print('/');
    if (startGPSTime.date < 10)//day
    {
      logFile.print("0");
    }
    logFile.print(startGPSTime.date);
    logFile.print(' ');
    if (startGPSTime.hours < 10)//hour
    {
      logFile.print("0");
    }
    logFile.print(startGPSTime.hours);
    logFile.print(':');
    if (startGPSTime.minutes < 10)//minute
    {
      logFile.print("0");
    }
    logFile.print(startGPSTime.minutes);
    logFile.print(':');
    if (startGPSTime.seconds < 10)//seconds
    {
      logFile.print("0");
    }
    logFile.print(startGPSTime.seconds);
    logFile.print('.');
    logFile.print(startGPSTimeMS);//cs
    logFile.print(',');

    // Print GPS Status
    logFile.print(currentFix.status);// Fix Status Code
    logFile.println();
    logFile.close();
    // If the first log was recorded
    if (firstDistLog == 0) {
      startLocation = currentFix;
      firstDistLog = 1;
    }
    count++;
    return 1; // Return success
  }
  else {
    strip.setPixelColor(LED1, red);// Set second light to Red to indicate unable to open/save to file
    return 0; // If we failed to open the file, return fail
  }
}//End logGPSData

byte printTest()
{
  logFile = SD.open(filename, FILE_WRITE);
  if ( ! logFile ) {
    perror("3");
    strip.setPixelColor(LED1, red); // Set second light to blue to indicate File is able to be written
    strip.show();
    return 0;
  }
  else if (logFile) // If the log file opened print to the file
  {
    logFile.print("Calibration: ");
    strip.setPixelColor(LED1, blue); // Set second light to blue to indicate File is able to be written
    strip.show();
  }
  logFile.close();
  return 1;
}//End printTest

  

static void printCalibration()
{
  logFile = SD.open(filename, FILE_WRITE);
  if ( ! logFile ) {
    perror("3");
    strip.setPixelColor(LED2, red); // Set third light to blue to indicate calibration measurement recorded
    strip.show();
  }
  else if (logFile) // If the log file opened print to the file
  {
    logFile.print(calibration);
    logFile.println();
    // Print the header
    int i = 0;
    for (; i < LOG_COLUMN_COUNT; i++)
    {
      logFile.print(log_col_names[i]);
      if (i < LOG_COLUMN_COUNT - 1)
        logFile.print(',');
      else
        logFile.println();
    }
    logFile.close();
    firstColLog = 1;
    strip.setPixelColor(LED2, blue); // Set third light to blue to indicate calibration measurement recorded
    strip.show();
  }
} //End printCalibration


////////////////////////////
///////////////////////////
/// LIDAR SETTINGS  //////
/////////////////////////
////////////////////////
//---------------------------------------------------------------------
// Read Single Distance Measurement
//
// This is the simplest form of taking a measurement. This is a
// blocking function as it will not return until a range has been
// taken and a new distance measurement can be read.
//---------------------------------------------------------------------
uint8_t distanceSingle(uint16_t * distance)
{
    // 1. Trigger range measurement.
    myLidarLite.takeRange();

    // 2. Wait for busyFlag to indicate device is idle.
    myLidarLite.waitForBusy();

    // 3. Read new distance data from device registers
    *distance = myLidarLite.readDistance();

    return 1;
}

//---------------------------------------------------------------------
// Read Single Distance Measurement using Trigger / Monitor Pins
//
// This is the simplest form of taking a measurement. This is a
// blocking function as it will not return until a range has been
// taken and a new distance measurement can be read. Instead of using
// the STATUS register to poll for BUSY, this function uses a
// GPIO pin on the LIDAR-Lite to monitor the BUSY flag.
//---------------------------------------------------------------------
//uint8_t distanceSingleGpio(uint16_t * distance)
//{
//    // 1. Trigger range measurement.
//    myLidarLite.takeRangeGpio(TriggerPin, MonitorPin);
//
//    // 2. Wait for busyFlag to indicate device is idle.
//    myLidarLite.waitForBusyGpio(MonitorPin);
//
//    // 3. Read new distance data from device registers
//    *distance = myLidarLite.readDistance();
//
//    return 1;
//}

//---------------------------------------------------------------------
// Read Continuous Distance Measurements
//
// The most recent distance measurement can always be read from
// device registers. Polling for the BUSY flag in the STATUS
// register can alert the user that the distance measurement is new
// and that the next measurement can be initiated. If the device is
// BUSY this function does nothing and returns 0. If the device is
// NOT BUSY this function triggers the next measurement, reads the
// distance data from the previous measurement, and returns 1.
//---------------------------------------------------------------------
uint8_t distanceContinuous(uint16_t * distance)
{
    uint8_t newDistance = 0;

    // Check on busyFlag to indicate if device is idle
    // (meaning = it finished the previously triggered measurement)
    if (myLidarLite.getBusyFlag() == 0)
    {
        // Trigger the next range measurement
        myLidarLite.takeRange();

        // Read new distance data from device registers
        *distance = myLidarLite.readDistance();

        // Report to calling function that we have new data
        newDistance = 1;
    }
    else
    {
      delay(5);
      if (myLidarLite.getBusyFlag() == 0)
      {
        // Trigger the next range measurement
        myLidarLite.takeRange();

        // Read new distance data from device registers
        *distance = myLidarLite.readDistance();

        // Report to calling function that we have new data
        newDistance = 1;
      }
    }
    return newDistance;
}

//---------------------------------------------------------------------
// Read Continuous Distance Measurements using Trigger / Monitor Pins
//
// The most recent distance measurement can always be read from
// device registers. Polling for the BUSY flag using the Monitor Pin
// can alert the user that the distance measurement is new
// and that the next measurement can be initiated. If the device is
// BUSY this function does nothing and returns 0. If the device is
// NOT BUSY this function triggers the next measurement, reads the
// distance data from the previous measurement, and returns 1.
//---------------------------------------------------------------------
//uint8_t distanceContinuousGpio(uint16_t * distance)
//{
//    uint8_t newDistance = 0;
//
//    // Check on busyFlag to indicate if device is idle
//    // (meaning = it finished the previously triggered measurement)
//    if (myLidarLite.getBusyFlagGpio(MonitorPin) == 0)
//    {
//        // Trigger the next range measurement
//        myLidarLite.takeRangeGpio(TriggerPin, MonitorPin);
//
//        // Read new distance data from device registers
//        *distance = myLidarLite.readDistance();
//
//        // Report to calling function that we have new data
//        newDistance = 1;
//    }
//    return newDistance;
//}
