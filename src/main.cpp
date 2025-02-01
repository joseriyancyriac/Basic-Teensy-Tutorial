#include <Arduino.h>
#include <AccelStepper.h>

// ==================== PIN ASSIGNMENTS ====================
constexpr uint8_t STEP_PIN = 4;
constexpr uint8_t DIR_PIN = 5;
constexpr uint8_t ENABLE_PIN = 6;
constexpr uint8_t HOME_SWITCH_PIN = 2; // Top limit switch

// ==================== CONSTANTS ====================
constexpr uint16_t MAX_SPEED = 480;     // Steps/sec (normal movement)
constexpr uint16_t HOMING_SPEED = 200;  // Steps/sec (slow for homing)
constexpr uint16_t ACCELERATION = 240;  // Steps/sec²
constexpr int32_t MAX_STEPS_DOWN = 588; // Max displacement from home
constexpr int32_t BACKOFF_STEPS = 10;   // Steps to back off after hitting limit
constexpr uint8_t TOF_THRESHOLD = 8;    // Required clearance in cm
constexpr float TOF_DEAD_ZONE = 1;      // Prevent jitter (cm)

// ==================== GLOBALS ====================
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// ==================== TF Mini LiDAR Configuration ====================
HardwareSerial &lidarSerial = Serial3; // Use Serial2 (RX1=pin 7, TX1=pin 8)

// Packet structure constants
const uint8_t HEADER_BYTE = 0x59;
const uint8_t PACKET_SIZE = 9;

// Data storage
uint8_t lidarBuffer[PACKET_SIZE];
int16_t distance = -1; // -1 = invalid measurement
int16_t strength = -1;
bool validData = false;

bool isHomed = false;
float current_TOF_distance;

// ==================== FUNCTION PROTOTYPES ====================
void homeMotor();
float getTOFDistance();

void setup()
{
  Serial.begin(115200);
  lidarSerial.begin(115200); // TF Mini default baud rate

  // while (!Serial); // Wait for Trigger

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // Enable motor
  pinMode(HOME_SWITCH_PIN, INPUT_PULLUP);

  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(ACCELERATION);
  stepper.setCurrentPosition(0);

  Serial.println("System starting up... Auto-homing.");
  homeMotor(); // **Auto-start homing sequence**
}

// ==================== HOMING ROUTINE ====================
void homeMotor()
{
  Serial.println("Homing: Moving UP until limit switch is pressed...");

  digitalWrite(ENABLE_PIN, LOW);

  // Move up until the limit switch is triggered
  stepper.setSpeed(-HOMING_SPEED);
  while (digitalRead(HOME_SWITCH_PIN) != LOW)
  {
    stepper.runSpeed();
  }

  // Back off slightly
  stepper.move(BACKOFF_STEPS);
  while (stepper.run())
  {
  }

  stepper.setCurrentPosition(10);
  isHomed = true;
  Serial.println("Homing complete! Position set.");
}

// ==================== TOF-CONTROLLED MOVEMENT ====================
void loop()
{
  if (!isHomed)
    return; // **Skip loop execution until homing is done**

  current_TOF_distance = getTOFDistance();

  if (current_TOF_distance == -2)
  {
    Serial.println("TOF ERROR: Invalid reading!");
    return;
  }

  Serial.print("TOF Distance: ");
  Serial.print(current_TOF_distance);
  Serial.println(" cm");

  // If above threshold, move down
  if (current_TOF_distance > TOF_THRESHOLD + TOF_DEAD_ZONE)
  {
    stepper.setSpeed(MAX_SPEED);
    stepper.run();
  }
  // If below threshold, move up
  else if (current_TOF_distance < TOF_THRESHOLD - TOF_DEAD_ZONE)
  {
    if (digitalRead(HOME_SWITCH_PIN) == LOW)
    {
      Serial.println("Limit switch triggered! Stopping upward motion.");
      return;
    }
    stepper.setSpeed(-MAX_SPEED);
    stepper.run();
  }
}

// ==================== TOF SENSOR READING ====================
float getTOFDistance()
{
  validData = false;

  if (lidarSerial.available() >= PACKET_SIZE)
  {
    // Read entire packet
    lidarSerial.readBytes(lidarBuffer, PACKET_SIZE);

    // Verify packet headers
    if (lidarBuffer[0] == HEADER_BYTE && lidarBuffer[1] == HEADER_BYTE)
    {
      // Calculate checksum
      uint16_t checksum = 0;
      for (uint8_t i = 0; i < 8; i++)
      {
        checksum += lidarBuffer[i];
      }

      // Validate checksum
      if ((checksum & 0xFF) == lidarBuffer[8])
      {
        distance = lidarBuffer[2] | (lidarBuffer[3] << 8);
        strength = lidarBuffer[4] | (lidarBuffer[5] << 8);
        validData = true;
      }
    }
  }
  return distance; // in cm
}
