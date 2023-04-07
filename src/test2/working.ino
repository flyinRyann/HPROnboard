/*
  SD Card Ports:
    - SD card attached to SPI bus on Arduino Nano as follows:
      - MOSI: Pin 11
      - MISO: Pin 12
      - CLK: Pin 13
      - CS/SS: Pin 10

  Sound Sensor Ports:
    - Sound Sensor 1 to Arduino Nano.
      - A0: Pin A6
      - +: 3V3
      - G: GND
    - Sound Sensor 2 to Arduino Nano.
      - A0: Pin A7
      - +: 3V3
      - G: GND
*/



// SD Library:
#include <SPI.h>
#include <SD.h>

// ---------- PIN DEFINTIONS ----------
#define SOUND_SENSOR_1 A6 // Sound sensor pin 1.
#define SOUND_SENSOR_2 A7 // Sound sensor pin 2.

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
//#define OUTPUT_READABLE_ACCELGYRO

#define LED_PIN 13
//bool blinkState = false;

#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp; // I2C


float currentAltitude = 96;

// Ryan's pins.
const int chipSelect = 10; // CS/SS pin for SD logger.
const int buzzer = 9; // Buzzer control pin.
const int sampleWindow = 50; // Sample window to collect sound sensor information in ms.
unsigned int sample;
long int t1 = millis();



// Set-Up Stage:
void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
  #endif

  Serial.begin(9600); // Open serial communication.

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  // -----------
  // configure Arduino LED for
  pinMode(LED_PIN, OUTPUT);


  while ( !Serial ) delay(100);   // wait for native usb
  Serial.println(F("BMP280 test")); 
  unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin();
  //  if (!status) {
  //    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
  //                      "try a different address!"));
  //    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
  //    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
  //    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
  //    Serial.print("        ID of 0x60 represents a BME 280.\n");
  //    Serial.print("        ID of 0x61 represents a BME 680.\n");
  //    while (1) delay(10);
  //  }



   /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  // ---------
  
  // BUZZER SETUP:
  pinMode(buzzer, OUTPUT);

  // SOUND SENSOR SETUP:
  pinMode(SOUND_SENSOR_1, INPUT);
  pinMode(SOUND_SENSOR_2, INPUT);

  // FUNCTION CALLS:
  sDCardInitialiser(); // Initialise SD Card for reading and writing.
  
  // SET-UP TEXT FILE LABELS:
  File dataFile = SD.open("yourmum.txt", FILE_WRITE);
  String dataString = "Time;Sound Sensor 1;Sound Sensor 2;Temperature;Pressure;Altitude;Height;Acceleration X;Acceleration Y;Acceleration Z;Gyro X;Gyro Y;Gyro Z;";

  // If the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }
  // If the file isn't open, pop up an error:
  else {
    Serial.println("Error opening datalog.txt.");
  }
}

void loop() {
  // Run function to measure sound and display on serial monitor.
  int soundSensor1DB = measureSound(SOUND_SENSOR_1, "Sensor 1");
  int soundSensor2DB = measureSound(SOUND_SENSOR_2, "Sensor 2");

  long int t2 = millis();
  int time = (t2 - t1);

  // Make a string for assembling the data to log:
  String dataString = "";
  int dataArray[] = {time, soundSensor1DB, soundSensor2DB};

  for(int i = 0; i < 3; i++){
    dataString += dataArray[i];
    dataString += ";";  
  }

  dataString += bmpFunc();
  dataString += mpuFunc();


  Serial.println(dataString);


  // WRITING DATA TO FILE
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("yourmum.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("Error opening datalog.txt.");
  }

  

}

// ---------- BUZZER ----------
/* 
** Description:
**    - Buzzer Trigger Function
**    - Send a 1KHz sound signal for 1 second at a time with a pause period of one second. This
**      will be useful in the process of retrieving the rocket.  
*/
void runBuzzer(){
  tone(buzzer, 1000); // Send 1KHz sound signal.
  delay(1000);        // Delay and continue the sound for 1 sec.
  noTone(buzzer);     // Stop sound the sound.
  delay(1000);        // Delay and stop the sound for 1 sec.
}

// ---------- SOUND SENSOR ----------

int measureSound(int SENSOR_PIN, String sensorName){
  // millis() used to return the number of milliseconds at the time, the Arduino board begins running the current program.
  unsigned long startMillis= millis();                    // Start of sample window.
  float peakToPeak = 0;                                   // Peak-to-peak level.

  unsigned int signalMax = 0;                             //  Minimum value.
  unsigned int signalMin = 1024;                          //  Maximum value.

  // Collect data for 50 mS:
  while (millis() - startMillis < sampleWindow){
     sample = analogRead(SENSOR_PIN);                     // Get reading from microphone.
     if (sample < 1024)                                   // Toss out spurious readings.
     {
        if (sample > signalMax)
        {
           signalMax = sample;                            // Save just the max levels.
        }
        else if (sample < signalMin)
        {
           signalMin = sample;                            // Save just the min levels.
        }
     }
  }

  peakToPeak = signalMax - signalMin;                     // max - min = peak-peak amplitude
  int db = map(peakToPeak,20,900,49.5,90);                // Calibrate for deciBels.

  return db;
//  Serial.print("Loudness of " + sensorName + ":");
//  Serial.print(db);
//  Serial.println("dB");
  
 delay(500); // Display the measurement every 0.5s. 
}

// ---------- SD CARD LOGGER ---------
void sDCardInitialiser(){
  while (!Serial) {
    ; // Wait for serial port to connect.
  }
  // SD Card Initialisation Stage.
  Serial.print("Initializing SD card...");

  // See if the card is present and can be initialized.
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present.");
    while (1);
  }
  Serial.println("Card initialized.");
}

// ----- BMP AND IMU

String bmpFunc() {
  String dataString = "";
//  dataString += toString(bmp.readTemperature()) + ";" + toString(bmp.readPressure()) + ";" + toString(bmp.readAltitude(1000)) + ";" + toString(bmp.readAltitude(1000) - currentAltitude)) + ";";
  float val = bmp.readAltitude(1000) - currentAltitude;
  float dataArray[] = {bmp.readTemperature(),bmp.readPressure(),bmp.readAltitude(1000),val};

  for(int i = 0; i < 4; i++){
    dataString += dataArray[i];
    dataString += ";";  
  }
  return dataString;
}


String mpuFunc() {
  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//  #ifdef OUTPUT_READABLE_ACCELGYRO
  String dataString = "";

//  dataString += ax + ";" + ay + ";" + az + ";" + gx + ";" + gy + ";" + gz + ";";
  int dataArray[] = {ax, ay, az, gx, gy, gz};

  for(int i = 0; i < 6; i++){
    dataString += dataArray[i];
    dataString += ";";  
  }
  return dataString;
}
