#include <SD.h>
#include <SPI.h>
#include <Stepper.h>
#include <Wire.h>
#include <MPU6050.h>


#define STEPPER_STEPS_PER_REVOLUTION 100

int currentPercentage = 0;

MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;

Stepper airBrakesStepper(STEPPER_STEPS_PER_REVOLUTION, 8, 9, 10, 11);
MPU6050 accelgyro;

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

  accelgyro.initialize();
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  logToFile("accelerationX,accelerationY,accelerationZ,gyroX,gyroY,gyroZ,temperature");

  Serial.println("Finished intialisation.");
}


void loop()
{
  int ax, ay, az, gx, gy, gz, temp;
  char dataString[1024];
  // Inbuilt LED
  digitalWrite(LED_BUILTIN, HIGH);

  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  temp = accelgyro.getTemperature();

  sprintf(dataString, "%i,%i,%i,%i,%i,%i,%i", ax, ay, az, gx, gy, gz, temp);
  logToFile(dataString);

  delay(1000);
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
