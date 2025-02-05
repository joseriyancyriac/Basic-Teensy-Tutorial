#include <Arduino.h>
//#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// BNO055 Settings
#define BNO055_I2C_ADDRESS 0x28  // Default I2C address
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_I2C_ADDRESS, &Wire1);

void setup(void) 
{
  Serial.begin(115200);
  while(!Serial); // Wait for serial connection
  
  // Initialize I2C1 (second I2C bus) with proper Teensy 4.0 syntax
  Wire1.begin();     // No parameters needed
  
  Serial.println("\nBNO055 Test Started");

  if(!bno.begin()) {
    Serial.println("BNO055 not detected!");
    while(1);
  }
  
  bno.setExtCrystalUse(true);
  Serial.println("BNO055 initialized");
  delay(1000);
}

void loop(void) 
{
  // Get Euler angles (degrees)
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  
  // Get linear acceleration (m/s²)
  imu::Vector<3> linaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  
  // Get gravity vector (m/s²)
  imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

  Serial.println("\n--- Sensor Data ---");
  
  // Euler angles
  Serial.print("Orientation - ");
  Serial.print("Roll: ");
  Serial.print(euler.z(), 1);
  Serial.print("°\tPitch: ");
  Serial.print(euler.y(), 1);
  Serial.print("°\tYaw: ");
  Serial.print(euler.x(), 1);
  Serial.println("°");

  // Linear acceleration
  Serial.print("Linear Acc - ");
  Serial.print("X: ");
  Serial.print(linaccel.x(), 2);
  Serial.print(" m/s²\tY: ");
  Serial.print(linaccel.y(), 2);
  Serial.print(" m/s²\tZ: ");
  Serial.print(linaccel.z(), 2);
  Serial.println(" m/s²");

  // Gravity vector
  Serial.print("Gravity - ");
  Serial.print("X: ");
  Serial.print(gravity.x(), 2);
  Serial.print(" m/s²\tY: ");
  Serial.print(gravity.y(), 2);
  Serial.print(" m/s²\tZ: ");
  Serial.print(gravity.z(), 2);
  Serial.println(" m/s²");

  // Add some delay between readings
  delay(250);
}