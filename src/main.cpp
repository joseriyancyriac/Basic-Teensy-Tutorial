#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ==================== OLED DISPLAY CONFIG ====================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1       // Reset pin not used
#define SCREEN_ADDRESS 0x3C // I2C address for most OLEDs

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ==================== PIN ASSIGNMENTS ====================
constexpr uint8_t STEP_PIN = 4;         // Step pulse output
constexpr uint8_t DIR_PIN = 5;          // Direction control
constexpr uint8_t ENABLE_PIN = 6;       // Motor enable (active LOW)
constexpr uint8_t HOME_SWITCH_PIN = 2;  // Top limit switch
constexpr uint8_t EMERGENCY_SWITCH_PIN = 3; // Emergency brake switch (assumed active LOW)
// BUTTON_PIN not used in this version

// ==================== CONSTANTS ====================
constexpr uint16_t MAX_SPEED = 400;               // Steps per second
constexpr uint16_t HOMING_SPEED = 200;            // Steps per second for homing
constexpr int32_t BACKOFF_STEPS = 10;             // Steps to back off after hitting limit
// Target clearance is now variable for dynamic adjustment.
uint8_t tof_threshold = 10;                       // Default target clearance in cm
constexpr float TOF_DEAD_ZONE = 1;                // Dead zone in cm
constexpr uint16_t DISPLAY_UPDATE_INTERVAL = 200; // ms update rate

// Override delay: if the reading stays above (tof_threshold + TOF_DEAD_ZONE)
// for this many milliseconds, force downward motion until clearance reaches the target.
const unsigned long OVERRIDE_DELAY = 5000; // 5000 ms (5 seconds)

// ==================== GLOBAL VARIABLES ====================
// Compute step interval (in microseconds) from MAX_SPEED
const unsigned long stepInterval = 1000000UL / MAX_SPEED;
unsigned long lastStepTime = 0;

// Variables for override functionality:
unsigned long overrideTimerStart = 0; // Time when reading first exceeded threshold+dead zone
bool overrideActive = false;

// ==================== TF Mini LiDAR CONFIGURATION ====================
// Two sensors: front and rear.
HardwareSerial &frontLidar = Serial3; // Front sensor on Serial3
HardwareSerial &rearLidar  = Serial2;  // Rear sensor on Serial2

// Packet structure constants (same for both sensors)
const uint8_t HEADER_BYTE = 0x59;
const uint8_t PACKET_SIZE = 9;

// Data storage for front sensor:
uint8_t frontLidarBuffer[PACKET_SIZE];
int16_t front_distance = -1; // in cm; -1 indicates an invalid reading

// Data storage for rear sensor:
uint8_t rearLidarBuffer[PACKET_SIZE];
int16_t rear_distance = -1;  // in cm; -1 indicates an invalid reading

// Buffer variables for last valid readings:
float last_valid_front = -1;
float last_valid_rear = -1;

// For display update:
bool displayConnected = false;
uint32_t lastDisplayUpdate = 0;

// Regular variable:
bool isHomed = false;

// Use the sensor reading directly (no smoothing)
float final_distance = -1;

// ------------------ Hysteresis State ------------------
// motion_state: 1 = moving downward, -1 = moving upward, 0 = stable.
int motion_state = 0;

// ==================== FUNCTION PROTOTYPES ====================
void homeMotor();
float getFrontTOFDistance();
float getRearTOFDistance();
void updateDisplay(float distance);
void stepMotor(bool direction); // Generic pulse function (true = down, false = up)
void genericStepperControl(bool moveDown);

//
// stepMotor: Pulses the STEP pin after setting the direction via DIR_PIN.
//
void stepMotor(bool direction) {
  digitalWrite(DIR_PIN, direction ? HIGH : LOW);
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(10); // Pulse width (adjust if needed)
  digitalWrite(STEP_PIN, LOW);
}

//
// genericStepperControl: Issues a step pulse if enough time has passed.
//
void genericStepperControl(bool moveDown) {
  unsigned long now = micros();
  if (now - lastStepTime >= stepInterval) {
    lastStepTime = now;
    stepMotor(moveDown);
  }
}

void setup() {
  // Initialize serial ports.
  Serial.begin(115200);
  frontLidar.begin(115200);
  rearLidar.begin(115200);

  // Set pin modes.
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // Enable motor driver (active LOW)
  pinMode(HOME_SWITCH_PIN, INPUT_PULLUP);
  pinMode(EMERGENCY_SWITCH_PIN, INPUT); // Set emergency switch pin

  // Initialize OLED display.
  if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    displayConnected = true;
  } else {
    Serial.println("OLED display not found");
    displayConnected = false;
  }

  Serial.println("System starting up... Auto-homing.");
  homeMotor(); // Execute homing sequence
}

//
// homeMotor: Moves upward (direction = false) until the limit switch is triggered,
// then backs off a fixed number of steps.
//
void homeMotor() {
  Serial.println("Homing: Moving UP until limit switch is pressed...");

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 5);
  display.println("System Online");
  display.setTextSize(2);
  display.setCursor(0, 30);
  display.print("Homing...");
  display.display();

  // Move upward until limit switch is triggered.
  while (digitalRead(HOME_SWITCH_PIN) != LOW) {
    genericStepperControl(false); // false: upward motion
  }
  Serial.println("Limit switch hit. Stopping homing upward motion.");

  // Back off: move downward a fixed number of steps.
  for (int i = 0; i < BACKOFF_STEPS; i++) {
    while (micros() - lastStepTime < stepInterval) { }
    lastStepTime = micros();
    stepMotor(true); // true: downward motion
  }
  isHomed = true;
  Serial.println("Homing complete!");

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.print("Ready!");
  display.display();
}

//
// getFrontTOFDistance: Reads and parses the front LiDAR sensor value.
float getFrontTOFDistance() {
  front_distance = -1;
  if (frontLidar.available() >= PACKET_SIZE) {
    frontLidar.readBytes(frontLidarBuffer, PACKET_SIZE);
    if (frontLidarBuffer[0] == HEADER_BYTE && frontLidarBuffer[1] == HEADER_BYTE) {
      uint16_t checksum = 0;
      for (uint8_t i = 0; i < 8; i++) {
        checksum += frontLidarBuffer[i];
      }
      if ((checksum & 0xFF) == frontLidarBuffer[8]) {
        front_distance = frontLidarBuffer[2] | (frontLidarBuffer[3] << 8);
      }
    }
  }
  return (float)front_distance;
}

//
// getRearTOFDistance: Reads and parses the rear LiDAR sensor value.
float getRearTOFDistance() {
  rear_distance = -1;
  if (rearLidar.available() >= PACKET_SIZE) {
    rearLidar.readBytes(rearLidarBuffer, PACKET_SIZE);
    if (rearLidarBuffer[0] == HEADER_BYTE && rearLidarBuffer[1] == HEADER_BYTE) {
      uint16_t checksum = 0;
      for (uint8_t i = 0; i < 8; i++) {
        checksum += rearLidarBuffer[i];
      }
      if ((checksum & 0xFF) == rearLidarBuffer[8]) {
        rear_distance = rearLidarBuffer[2] | (rearLidarBuffer[3] << 8);
      }
    }
  }
  return (float)rear_distance;
}

//
// updateDisplay: Updates the OLED with the current clearance.
// In emergency mode, it displays "E1".
void updateDisplay(float distance) {
  if (!displayConnected || (millis() - lastDisplayUpdate < DISPLAY_UPDATE_INTERVAL))
    return;

  lastDisplayUpdate = millis();
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2);
  if (overrideActive) {
    display.print("E1");
  } else {
    display.print("Clearance:");
    display.setCursor(0, 30);
    display.setTextSize(2);
    display.print(distance);
    display.print(" cm");
    if (digitalRead(HOME_SWITCH_PIN) == LOW) {
      display.setCursor(0, 45);
      display.setTextSize(1);
      display.print("LIMIT HIT!!!");
    }
  }
  display.display();
}

//
// loop: Reads sensor values, buffers invalid readings, fuses the data,
// checks for incoming threshold updates from ROS2 (via Serial),
// and then commands the stepper motor using hysteresis logic with an override.
// In addition, if an emergency condition is detected via EMERGENCY_SWITCH_PIN,
// the code sends an emergency trigger ("E:1") over Serial, updates the display,
// and commands the moving base to move down to the home limit position.
void loop() {
  // --- Check for Emergency Brake Trigger ---
  if (digitalRead(EMERGENCY_SWITCH_PIN) == HIGH) { // Assuming active LOW means emergency
    Serial.println("E:1"); // Send emergency trigger over Serial
    // Update display with emergency indicator
    display.clearDisplay();
    display.setCursor(20, 25);
    display.setTextSize(3);
    display.print("E1");
    display.display();
    // Command the moving base to move downward until home limit is reached.
    while (digitalRead(HOME_SWITCH_PIN) != LOW) {
      genericStepperControl(false); // Force downward motion
    }
    // When home limit is reached, send clear emergency message.
    Serial.println("E:0");
    // Then exit loop (or wait here until emergency condition is cleared).
    return;
  }

  if (!isHomed)
    return;

  // --- Check for incoming threshold update via Serial (from ROS2) ---
  if (Serial.available() > 0) {
    int newThreshold = Serial.parseInt();
    if (newThreshold > 0 && newThreshold < 50) { // reasonable upper bound
      tof_threshold = newThreshold;
      //Serial.print("Updated TOF threshold to: ");
      //Serial.println(tof_threshold);
    }
  }

  float front_val = getFrontTOFDistance();
  float rear_val = getRearTOFDistance();

  // Buffer invalid readings.
  if (front_val == -1 && last_valid_front != -1) {
    front_val = last_valid_front;
  } else if (front_val != -1) {
    last_valid_front = front_val;
  }
  if (rear_val == -1 && last_valid_rear != -1) {
    rear_val = last_valid_rear;
  } else if (rear_val != -1) {
    last_valid_rear = rear_val;
  }

  // Calibrate the front sensor by subtracting 3 cm.
  float calibrated_front = front_val - 3.0;
  if (calibrated_front < 0) {
    calibrated_front = 0;
  }
  float effective_front = calibrated_front;

  bool validFront = (effective_front != -1);
  bool validRear = (rear_val != -1);
  float current_TOF_distance = -1;

  if (validFront && validRear) {
    // Use the lower (more conservative) value.
    if (effective_front > rear_val)
      current_TOF_distance = rear_val;
    else if (effective_front < rear_val)
      current_TOF_distance = effective_front;
    else
      current_TOF_distance = effective_front;
  } else if (validFront) {
    current_TOF_distance = effective_front;
  } else if (validRear) {
    current_TOF_distance = rear_val;
  }

  if (current_TOF_distance == -1) {
    Serial.println("TOF ERROR: Invalid reading from both sensors!");
    return;
  }

  // Use the sensor reading directly.
  final_distance = current_TOF_distance;

  //Serial.print("Front (cali): ");
  //Serial.print(calibrated_front);
  //Serial.print(" cm, Rear: ");
  //Serial.print(rear_val);
  //Serial.print(" cm, Final: ");
  //Serial.print(final_distance);
  //Serial.println(" cm");

  updateDisplay(final_distance);

  // --- Hysteresis & Override Logic ---
  unsigned long now = millis();
  // Normal override: if the clearance is above the target (tof_threshold)
  // (you might want to check against (tof_threshold + TOF_DEAD_ZONE) but here we check > threshold)
  if (final_distance > tof_threshold) {
    if (overrideTimerStart == 0) {
      overrideTimerStart = now;
    } else if (now - overrideTimerStart >= OVERRIDE_DELAY) {
      overrideActive = true;
      //Serial.println("Override active: forcing downward motion");
    }
  } else {
    overrideTimerStart = 0;
    overrideActive = false;
  }

  // If override is active, force downward motion until clearance <= tof_threshold.
  if (overrideActive && final_distance > tof_threshold) {
    if (motion_state != 1) {
      motion_state = 1;
      //Serial.println("Override: Switching to downward motion");
    }
    genericStepperControl(true);
    return;
  }

  // Normal hysteresis logic.
  if (final_distance > tof_threshold + TOF_DEAD_ZONE) {
    if (motion_state != 1) {
      motion_state = 1;
      //Serial.println("Switching to downward motion");
    }
    genericStepperControl(true);
  } else if (final_distance < tof_threshold - TOF_DEAD_ZONE) {
    if (digitalRead(HOME_SWITCH_PIN) == LOW) {
      //Serial.println("Limit switch triggered! Stopping upward motion.");
      return;
    }
    if (motion_state != -1) {
      motion_state = -1;
      //Serial.println("Switching to upward motion");
    }
    genericStepperControl(false);
  } else {
    if (motion_state != 0) {
      motion_state = 0;
      //Serial.println("Stable; no movement commanded");
    }
  }
}
