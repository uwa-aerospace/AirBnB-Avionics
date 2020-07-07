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


Adafruit_BME280 bme;
#define DELAY_BETWEEN_READINGS_MS 1000

#define STEPPER_STEPS_PER_REVOLUTION 100

// GPS
#define GPS_RX_PIN 7
#define GPS_TX_PIN 10

// BME 280
#define BME_280_SDA_PIN 11
#define BME_280_SCL_PIN 12
#define SEALEVELPRESSURE_HPA (1013.25)


int currentPercentage = 0;

Adafruit_MPU6050 mpu;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;

Stepper airBrakesStepper(STEPPER_STEPS_PER_REVOLUTION, 8, 9, 10, 11);

// GPS
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;
static const uint32_t GPSBaud = 9600;

typedef struct
{
  float time;
  float height;
  float vertical_speed;
  float vertical_acceleration;
  float temperature;
} DATA_POINT;

void setup()
{
  Serial.begin(9600);
  Serial.println("Initialising.");

  // Inbuilt LED
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Card failed, or not present");
    return;
  }
  Serial.println("SD card initialized.");



#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // Try to initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // GPS
  gpsSerial.begin(GPSBaud);

  // BME 280
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }


  logToFile("latitude,longitude,groundspeed,accelerationX,accelerationY,accelerationZ,gyroX,gyroY,gyroZ,temperature,altitude,humidity,pressure");
  Serial.println("Finished intialisation.");
}


void loop()
{
  static char dataString[1024];
  static double latitude, longitude, groundspeed;
  static float altitude, humidity, pressure;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  Serial.print(a.acceleration.x);
  Serial.print(a.acceleration.y);
  Serial.print(a.acceleration.z); // m/s/s
  Serial.print(g.gyro.x); // rad/s
  Serial.print(g.gyro.y);
  Serial.print(g.gyro.z);
  Serial.print(temp.temperature); // °C

  // Inbuilt LED
  digitalWrite(LED_BUILTIN, HIGH);

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
  //  temperature = bme.readTemperature()); // ° C

  pressure = bme.readPressure() / 100.0F; // hPa

  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

  humidity = bme.readHumidity(); // %

  sprintf(dataString, "%f,%f,%f,%i,%i,%i,%i,%i,%i,%f,%f,%f,%f",latitude,longitude,groundspeed,ax,ay,az,gx,gy,gz,temp.temperature,altitude,humidity,pressure);
  logToFile(dataString);

  delay(DELAY_BETWEEN_READINGS_MS);
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
