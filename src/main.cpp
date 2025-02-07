#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ==================== OLED DISPLAY CONFIG ====================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1       // Reset pin not used
#define SCREEN_ADDRESS 0x3C // I2C address for most OLEDs

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ==================== PIN ASSIGNMENTS ====================
constexpr uint8_t STEP_PIN = 4;       // Step pulse output
constexpr uint8_t DIR_PIN  = 5;       // Direction control
constexpr uint8_t ENABLE_PIN = 6;     // Motor enable (active LOW)
constexpr uint8_t HOME_SWITCH_PIN = 2; // Top limit switch
constexpr uint8_t BUTTON_PIN = 3;      // Homing trigger button

// ==================== CONSTANTS ====================
constexpr uint16_t MAX_SPEED = 480;               // Steps per second
constexpr uint16_t HOMING_SPEED = 200;            // Steps per second for homing
constexpr int32_t BACKOFF_STEPS = 10;             // Steps to back off after hitting limit
constexpr uint8_t TOF_THRESHOLD = 10;             // Target clearance in cm (e.g., 10 cm)
constexpr float TOF_DEAD_ZONE = 1;                // Dead zone in cm
constexpr uint16_t DISPLAY_UPDATE_INTERVAL = 200; // ms update rate

// ==================== GLOBAL VARIABLES ====================
// Compute step interval (in microseconds) from MAX_SPEED
const unsigned long stepInterval = 1000000UL / MAX_SPEED;
unsigned long lastStepTime = 0;

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

// For smoothing sensor readings:
float smoothed_distance = -1;  // Initialized on first valid reading

// ==================== FUNCTION PROTOTYPES ====================
void homeMotor();
float getFrontTOFDistance();
float getRearTOFDistance();
void updateDisplay(float distance, const char *status);
void stepMotor(bool direction);  // Generic pulse function (direction: true = down, false = up)
void genericStepperControl(bool moveDown);

//
// Step the motor by pulsing the STEP pin. Direction is set via DIR_PIN.
//
void stepMotor(bool direction) {
  digitalWrite(DIR_PIN, direction ? HIGH : LOW);
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(10); // Pulse width; adjust if needed.
  digitalWrite(STEP_PIN, LOW);
}

//
// Generic stepper control: checks if enough time has passed since the last step,
// and if so, issues a step pulse in the given direction (true = downward, false = upward).
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
  digitalWrite(ENABLE_PIN, LOW);  // Enable motor driver (active LOW)
  pinMode(HOME_SWITCH_PIN, INPUT_PULLUP);
  //pinMode(BUTTON_PIN, INPUT_PULLUP);

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
// Homing routine using generic stepper control.
// Moves upward (direction = false) until the limit switch is triggered,
// then backs off a fixed number of steps.
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

  // Step upward until the limit switch is triggered.
  while (digitalRead(HOME_SWITCH_PIN) != LOW) {
    genericStepperControl(false);  // false: upward motion
  }
  Serial.println("Limit switch hit. Stopping homing upward motion.");

  // Back off: move downward a fixed number of steps.
  for (int i = 0; i < BACKOFF_STEPS; i++) {
    while (micros() - lastStepTime < stepInterval) { }
    lastStepTime = micros();
    stepMotor(true);  // true: downward motion
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
// Read and parse the front LiDAR sensor reading.
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
  return (float)front_distance; // in cm
}

//
// Read and parse the rear LiDAR sensor reading.
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
  return (float)rear_distance; // in cm
}

//
// Update the OLED display with the current clearance and status message.
// 'status' can be "Upward slope", "Downward slope", or "Stable".
void updateDisplay(float distance, const char *status) {
  if (!displayConnected || (millis() - lastDisplayUpdate < DISPLAY_UPDATE_INTERVAL))
    return;
  
  lastDisplayUpdate = millis();
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.print("Clearance:");
  display.setCursor(0, 30);
  display.setTextSize(2);
  display.print(distance);
  display.print(" cm");
  
  // Display slope status on the bottom line.
  display.setCursor(0, 55);
  display.setTextSize(1);
  display.print(status);
  
  if (digitalRead(HOME_SWITCH_PIN) == LOW) {
    display.setCursor(0, 45);
    display.setTextSize(1);
    display.print("LIMIT HIT!!!");
  }
  display.display();
}

//
// Main loop: read both sensors, buffer invalid readings,
// combine sensor values according to selection logic, smooth the result,
// compute a slope status, and command the stepper motor accordingly.
void loop() {
  if (!isHomed)
    return; // Skip loop until homing is complete

  // Re-home if homing button is pressed.
  //if (digitalRead(BUTTON_PIN) == HIGH) {
    //Serial.println("Homing button pressed! Re-initializing...");
    //homeMotor();
    //return;
  //}

  // Get raw sensor values.
  float front_val = getFrontTOFDistance();
  float rear_val  = getRearTOFDistance();

  // Buffer: if reading is invalid (-1) and we have a previous valid reading, use it.
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

  // Since sensors are mounted in the same plane, we use the front reading directly.
  float effective_front = front_val;

  bool validFront = (effective_front != -1);
  bool validRear  = (rear_val != -1);
  float current_TOF_distance = -1;

  if (validFront && validRear) {
    // Use the lower (more conservative) clearance.
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

  // Apply smoothing filter (exponential moving average).
  const float alpha = 0.1;  // Smoothing factor (adjust as needed)
  if (smoothed_distance < 0) {
    smoothed_distance = current_TOF_distance;
  } else {
    smoothed_distance = alpha * current_TOF_distance + (1 - alpha) * smoothed_distance;
  }

  // Determine slope status.
  const char *status = "Stable";
  if (smoothed_distance > TOF_THRESHOLD + TOF_DEAD_ZONE) {
    status = "Downward slope";
  } else if (smoothed_distance < TOF_THRESHOLD - TOF_DEAD_ZONE) {
    status = "Upward slope";
  }

  Serial.print("Front (raw): ");
  Serial.print(front_val);
  Serial.print(" cm, Rear: ");
  Serial.print(rear_val);
  Serial.print(" cm, Smoothed: ");
  Serial.print(smoothed_distance);
  Serial.print(" cm, Status: ");
  Serial.println(status);

  updateDisplay(smoothed_distance, status);

  // Motor control: maintain clearance at TOF_THRESHOLD (10 cm) ± dead zone.
  if (smoothed_distance > TOF_THRESHOLD + TOF_DEAD_ZONE) {
    genericStepperControl(true);  // true: move downward
  } else if (smoothed_distance < TOF_THRESHOLD - TOF_DEAD_ZONE) {
    if (digitalRead(HOME_SWITCH_PIN) == LOW) {
      Serial.println("Limit switch triggered! Stopping upward motion.");
      return;
    }
    genericStepperControl(false); // false: move upward
  }
}
