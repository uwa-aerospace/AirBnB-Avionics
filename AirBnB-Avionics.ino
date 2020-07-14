#include <SD.h>
#include <SPI.h>
#include <Stepper.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

// General constants
#define DELAY_BETWEEN_READINGS_MS 250
#define AIR_BRAKES_ON_DELAY 5000
#define STEPPER_STEPS_PER_REVOLUTION 100
#define TOTAL_ACCEL_TO_INDICATE_LAUNCH 10.0

// GPS pins and constants
#define GPS_RX_PIN 9
#define GPS_TX_PIN 10
#define GPS_BAUD_RATE 9600

// BMP 280 constants
#define SEALEVELPRESSURE_HPA 1019.66

// MPU6050 - accelerometer & gyroscope
Adafruit_MPU6050 mpu;

// BMP280 - pressure, temperature & altitude estimation
Adafruit_BMP280 bmp;

// GPS - latitude, longitude, groundspeed
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;

// Air brakes stepper motor with pins
Stepper airBrakesStepper(STEPPER_STEPS_PER_REVOLUTION, 8, 9, 10, 11);

unsigned long launchTime = 0;
int currentAirBrakesPercentage = 0;
bool launched = false;

void setup()
{
  Serial.begin(9600);
  Serial.println("--- Air BnB Air Brakes test flight ---\n");
  Serial.println("Beginning initialisation.");
  Serial.println("-------------------------");

  // see if the card is present and can be initialized:
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("x\tERROR: SD Card failed, or not present");
    exit(-1);
  }
  Serial.println("-\tSD card initialized.");


  Wire.begin();
  Serial.println("-\tI2C Wire initialized.");

  // Try to initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("x\tERROR: Failed to find MPU6050 chip");
    exit(-1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("-\tMPU6050 (accelerometer & gyro) initialized.");

  // GPS
  gpsSerial.begin(GPS_BAUD_RATE);
  Serial.println("-\tGPS Serial initialized.");

  // BMP 280
  if (!bmp.begin(0x76)) {
    Serial.println("x\tERROR: Failed to find BMP280 chip");
    exit(-1);
  }
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  Serial.println("-\tBMP280 (pressure, temperature and (rough) altitude) initialized.");

  logToFile("latitude,longitude,groundspeed,accelerationX,accelerationY,accelerationZ,gyroX,gyroY,gyroZ,temperature,altitude,pressure(hPa)");
  Serial.println("Finished intialisation.");
  Serial.println("-----------------------\n");
}


void loop()
{
  static double latitude, longitude, groundspeed;
  static float altitude, temperature, pressure;

  char dataString[2048];
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


  //   BMP 280
  temperature = bmp.readTemperature();
  pressure = bmp.readPressure() / 100;
  altitude =  bmp.readAltitude(SEALEVELPRESSURE_HPA);

  // Log data to SD card
  sprintf(dataString, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", latitude, longitude, groundspeed, a.acceleration.x, a.acceleration.y, a.acceleration.z, g.gyro.x, g.gyro.y, g.gyro.z, temperature, altitude, pressure);
  logToFile(dataString);


  // Activate air brakes a certain time after launch has occurred
  if (!launched) {
    launched = sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2)) > TOTAL_ACCEL_TO_INDICATE_LAUNCH;
    if (launched) {
      launchTime = millis();
      char logText[1024];
      sprintf(logText, "# Launch detected at time: %lu", launchTime);
      logToFile(logText);
    }
  } else {
    if (millis() - launchTime > AIR_BRAKES_ON_DELAY) {
      setAirBrakes(100);
    }
  }


  delay(DELAY_BETWEEN_READINGS_MS);
}


void setAirBrakes(int percentage) {
  int toChange = percentage - currentAirBrakesPercentage; // Determine how much to move & which way
  airBrakesStepper.step((int) (toChange * STEPPER_STEPS_PER_REVOLUTION) / 100);
  currentAirBrakesPercentage = percentage;

  char logText[1024];
  sprintf(logText, "# Air brakes set to percentage: %i%%", percentage);
  logToFile(logText);
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
    Serial.printf("error opening datalog.txt. Cannot write: %s\n", text); // Error with file
  }
}
