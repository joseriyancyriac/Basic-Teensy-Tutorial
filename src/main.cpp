#include <Arduino.h>
#include <AccelStepper.h>

// ==================== PIN ASSIGNMENTS ====================
constexpr uint8_t STEP_PIN = 4;
constexpr uint8_t DIR_PIN = 5;
constexpr uint8_t ENABLE_PIN = 6;
constexpr uint8_t HOME_SWITCH_PIN = 2; // Top limit switch

// ==================== CONSTANTS ====================
constexpr uint16_t MAX_SPEED = 1000;      // Steps/sec (normal movement)
constexpr uint16_t HOMING_SPEED = 200;   // Steps/sec (slow for homing)
constexpr uint16_t ACCELERATION = 400;   // Steps/sec²
constexpr int32_t MAX_STEPS_DOWN = 600; // Max displacement from home
constexpr int32_t BACKOFF_STEPS = 10;    // Steps to back off after hitting limit

// ==================== GLOBALS ====================
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
bool isHomed = false;

void setup() {
  Serial.begin(9600);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // Motor ALWAYS ENABLED (to hold position)
  pinMode(HOME_SWITCH_PIN, INPUT_PULLUP);

  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(ACCELERATION);
  stepper.setCurrentPosition(0);

  Serial.println("System ready. Send 'h' to home, 'w'/'s' to move.");
}

// --------------- Improved Homing Routine ---------------
void homeMotor() {
  Serial.println("Homing: Moving UP until limit switch is pressed...");
  
  digitalWrite(ENABLE_PIN, LOW);
  // Stage 1: Fast approach to limit switch
  stepper.setSpeed(-HOMING_SPEED); // Move UP (adjust sign based on wiring)
  while (digitalRead(HOME_SWITCH_PIN) != LOW) {
    stepper.runSpeed();
  }

  // Stage 2: Back off slightly (to release the switch)
  stepper.move(BACKOFF_STEPS);
  while (stepper.run()) {}

  // Set home position (0) and reset software limits
  stepper.setCurrentPosition(0);
  isHomed = true;
  Serial.println("Homed! Position.");
}

// --------------- Motion Control ---------------
void moveToTarget(int32_t target) {
  
  digitalWrite(ENABLE_PIN, LOW);

  if (!isHomed) {
    Serial.println("Home first!");
    return;
  }

  // Clamp target within allowed range [588, 0]
  target = constrain(target, 0, MAX_STEPS_DOWN);
  stepper.moveTo(target);

  // Block until movement completes
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  Serial.print("Reached position: ");
  Serial.println(stepper.currentPosition());
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();
    while (Serial.available()) Serial.read();

    switch (command) {
      case 'h': // Home
        homeMotor();
        break;
      case 'w': // Move UP to home (0)
        moveToTarget(0);
        break;
      case 's': // Move DOWN to max displacement (-588)
        moveToTarget(MAX_STEPS_DOWN);
        break;
      case 'd': // Stop
        stepper.stop();
        digitalWrite(ENABLE_PIN, HIGH);
        Serial.println("Stopped.");
        break;
      default:
        Serial.println("Invalid command. Use 'h', 'w', or 's'.");
    }
  }
}