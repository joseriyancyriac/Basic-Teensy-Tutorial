#include <Arduino.h>
#include <AccelStepper.h>

// ==================== CONFIGURATION ====================
constexpr uint8_t STEP_PIN = 4;
constexpr uint8_t DIR_PIN = 5;
constexpr uint8_t ENABLE_PIN = 6;
constexpr uint8_t HOME_SWITCH_PIN = 2;

// LiDAR Configuration
HardwareSerial &fixedLidar = Serial2;  // Fixed base reference
HardwareSerial &movingLidar = Serial3; // Floor distance measurement

// Control Parameters
constexpr int16_t TARGET_DISTANCE = 26;     // 8 cm floor gap
constexpr int16_t MIN_DIFFERENCE = 4;      // Minimum fixed-moving difference
constexpr float STEPS_PER_CM = 29.4;       // 588 steps = 20cm -> 29.4 steps/cm
constexpr int16_t DEADBAND = 2;            // 🔥 Stop small movements
constexpr uint16_t MAX_ADJUSTMENT = 10;    // 🔥 Prevent excessive step corrections
constexpr uint16_t MOVE_SPEED = 500;       // Steps/sec
constexpr uint16_t ACCEL = 500;            // Steps/sec²
constexpr uint16_t SAFETY_COOLDOWN = 5000; // 🔥 Cooldown before next safety check

// ==================== SYSTEM STATE ====================
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
enum State { DISABLED, HOMING, ACTIVE, SAFETY_ADJUST, CALIBRATING };
State currentState = DISABLED;

// Calibration Data
float fixedReference = 0; // Fixed LiDAR reading at home
bool motorEnabled = false;      // Track motor enable state
bool settled = false;           // 🔥 Prevent unnecessary corrections
uint32_t lastSafetyAdjust = 0; // Timer for safety condition

// 🔧 **Declare function prototypes** 🔧
void performHoming();
int16_t getValidDistance(HardwareSerial &sensor);
void checkSafetyCondition();
void updatePositionControl();
void checkCalibration();
void moveToPosition(int targetSteps);
void disableMotor();

void setup()
{
  Serial.begin(115200);
  fixedLidar.begin(115200);
  movingLidar.begin(115200);

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH); // Start motor in disabled state
  pinMode(HOME_SWITCH_PIN, INPUT_PULLUP);

  stepper.setMaxSpeed(MOVE_SPEED);
  stepper.setAcceleration(ACCEL);

  Serial.println("🛑 System is DISABLED. Send 'h' to home.");
}

// ==================== MAIN LOOP ====================
void loop()
{
  // ✅ **READ USER INPUT FIRST**
  if (Serial.available())
  {
    char cmd = Serial.read();
    while (Serial.available()) Serial.read(); // Clear input buffer

    if (cmd == 'h') { 
      performHoming(); 
    }
    if (cmd == 'w' && currentState == ACTIVE) { 
      moveToPosition(0); 
    }
    if (cmd == 's' && currentState == ACTIVE) { 
      moveToPosition(-588); 
    }
    if (cmd == 'd') { 
      disableMotor(); 
    }
  }

  // ✅ **Only run stepper logic if motor is enabled**
  if (!motorEnabled) return;

  switch (currentState)
  {
  case HOMING:
    performHoming();
    break;

  case ACTIVE:
    checkSafetyCondition();
    stepper.run(); 
    break;

  case SAFETY_ADJUST:
    updatePositionControl();
    checkCalibration();
    if (stepper.distanceToGo() == 0)
    {
      currentState = ACTIVE;
      Serial.println("✅ Safety condition resolved");
    }
    stepper.run();
    break;

  case DISABLED:
    return;
  }
}

// ==================== LiDAR FUNCTIONS ====================
int16_t getValidDistance(HardwareSerial &sensor)
{
  static uint8_t buffer[9];
  if (sensor.available() >= 9)
  {
    sensor.readBytes(buffer, 9);
    if (buffer[0] == 0x59 && buffer[1] == 0x59)
    {
      uint16_t checksum = 0;
      for (uint8_t i = 0; i < 8; i++)
        checksum += buffer[i];
      if ((checksum & 0xFF) == buffer[8])
      {
        return buffer[2] | (buffer[3] << 8);
      }
    }
  }
  return -1;
}

// ==================== CONTROL LOGIC ====================
void performHoming()
{
  Serial.println("🏁 Homing: Moving UP until limit switch is pressed...");
  currentState = HOMING;
  motorEnabled = true;
  settled = false;
  digitalWrite(ENABLE_PIN, LOW); // Enable motor

  stepper.setSpeed(-100); // Slow approach
  while (digitalRead(HOME_SWITCH_PIN) != LOW)
  {
    stepper.runSpeed();
  }

  stepper.move(10); // Back off a little
  while (stepper.run());

  stepper.setCurrentPosition(0);
  fixedReference = getValidDistance(fixedLidar);

  Serial.println("✅ Homing complete. Zero position set.");
  Serial.println("🔄 System ready. Use 'w' to move up, 's' to move down.");
  currentState = ACTIVE;
}

// **✅ FIX: Stop Adjusting If Already Close to Target ✅**
void updatePositionControl()
{
  int16_t currentDist = getValidDistance(movingLidar);
  if (currentDist == -1) return;

  int16_t error = TARGET_DISTANCE - currentDist;

  // 🔥 **NEW: Do NOT adjust if already moving**
  if (stepper.distanceToGo() != 0) return;

  // 🔥 **NEW: Stop adjusting if within deadband**
  if (abs(error) <= DEADBAND)
  {
    if (!settled)
    {
      Serial.println("✅ Position is stable. No adjustment needed.");
      settled = true;
    }
    return;
  }

  settled = false;

  int32_t stepAdjustment = constrain(error * STEPS_PER_CM, -MAX_ADJUSTMENT, MAX_ADJUSTMENT);
  moveToPosition(stepper.currentPosition() + stepAdjustment);

  Serial.print("✅ Adjusting position by ");
  Serial.print(stepAdjustment);
  Serial.println(" steps");
}

// ✅ **Final Fix: Ensure Safety Adjustments Are Not Repeated**
void checkSafetyCondition()
{
  int16_t fixed = getValidDistance(fixedLidar);
  int16_t moving = getValidDistance(movingLidar);

  if (fixed == -1 || moving == -1) return;

  int16_t difference = fixed - moving;

  // ✅ **NEW: Stop repeated safety adjustments**
  if (difference >= MIN_DIFFERENCE) return;

  // ✅ **NEW: Wait before making another safety adjustment**
  if (millis() - lastSafetyAdjust < SAFETY_COOLDOWN) return;

  lastSafetyAdjust = millis();
  currentState = SAFETY_ADJUST;
  int16_t requiredSteps = (MIN_DIFFERENCE - difference) * STEPS_PER_CM;
  moveToPosition(stepper.currentPosition() + requiredSteps);
  Serial.println("⚠️ Safety adjustment triggered");
}

void checkCalibration()
{
  static uint32_t lastCheck = 0;
  if (millis() - lastCheck < 5000) return; 
  lastCheck = millis();

  float expectedFixed = fixedReference + (abs(stepper.currentPosition()) / STEPS_PER_CM);
  float actualFixed = getValidDistance(fixedLidar);

  if (abs(expectedFixed - actualFixed) > 2.0)
  { 
    Serial.println("⚠️ System drift detected!");
  }
}

// ==================== SAFE MOVEMENT FUNCTION ====================
void moveToPosition(int targetSteps)
{
  targetSteps = constrain(targetSteps, 0, 588);
  stepper.moveTo(targetSteps);
}

// ==================== DISABLE MOTOR FUNCTION ====================
void disableMotor()
{
  Serial.println("🛑 Motor disabled.");
  motorEnabled = false;
  currentState = DISABLED;
  digitalWrite(ENABLE_PIN, HIGH); // Disable motor driver
}
