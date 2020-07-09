#include <SD.h>
#include <SPI.h>
#include <Stepper.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>


#define DELAY_BETWEEN_READINGS_MS 100

#define STEPPER_STEPS_PER_REVOLUTION 100

// GPS
#define GPS_RX_PIN 10
#define GPS_TX_PIN 9
#define GPS_BAUD_RATE 9600

// BME 280
//#define BME_280_SDA_PIN 19
//#define BME_280_SCL_PIN 18
#define SEALEVELPRESSURE_HPA (1013.25)

int currentPercentage = 0;

Adafruit_MPU6050 mpu;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

Adafruit_BME280 bme;

// GPS
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;

Stepper airBrakesStepper(STEPPER_STEPS_PER_REVOLUTION, 8, 9, 10, 11);


void setup()
{
  Serial.begin(9600);
  Serial.println("Initialising.");

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("ERROR: Card failed, or not present");
    exit(-1);
  }
  Serial.println("SD card initialized.");



#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // Try to initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("ERROR: Failed to find MPU6050 chip");
    exit(-1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // GPS
  gpsSerial.begin(GPS_BAUD_RATE);

  // BME 280
//  if (!bme.begin(/*0x76*/)) {
//    Serial.println("ERROR: Could not find a valid BME280 sensor, check wiring!");
//    exit(-1);
//  }


  logToFile("latitude,longitude,groundspeed,accelerationX,accelerationY,accelerationZ,gyroX,gyroY,gyroZ,temperature,altitude,humidity,pressure");
  Serial.println("Finished intialisation.");
}


void loop()
{

  static double latitude, longitude, groundspeed;
  static float altitude, humidity, pressure;

  char dataString[1024];
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // GPS
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
    if (gps.location.isUpdated()) {
      // Only begins once a GPS fix is gained.
      latitude = gps.location.lat();
      longitude = gps.location.lng();
    }
    if (gps.speed.isUpdated()) {
      groundspeed = gps.speed.kmph();
    }
  }
  gpsSerial.flush();


  // BME 280
//  //  temperature = bme.readTemperature()); // ° C
//
//  pressure = bme.readPressure() / 100.0F; // hPa
//
//  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
//
//  humidity = bme.readHumidity(); // %

  sprintf(dataString, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", latitude, longitude, groundspeed, a.acceleration.x, a.acceleration.y, a.acceleration.z, g.gyro.x, g.gyro.y, g.gyro.z, temp.temperature, altitude, humidity, pressure);
  logToFile(dataString);
  delay(DELAY_BETWEEN_READINGS_MS);

  if (false /* REPLACE WITH AIR BRAKES ON CONDITION */) {
    setAirBrakes(100);
  }
}

void setAirBrakes(int percentage) {
  int toChange = percentage - currentPercentage; // Determine which way to move
  airBrakesStepper.step((int) (toChange * STEPPER_STEPS_PER_REVOLUTION) / 100);
  currentPercentage = percentage;
}

void logToFile(String text) {
  // Log the data

  File logFile = SD.open("AirBnB-flight-log.csv", FILE_WRITE);

  if (logFile) {
    logFile.println(text);
    logFile.close();
    Serial.printf("Written to file: %s\n", text);
  }
  else {
    Serial.println("error opening datalog.txt"); // Error with file
  }
}
