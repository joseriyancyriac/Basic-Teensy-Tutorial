# **Basic Teensy Stepper Motor & Limit Switch Control**  

## **📌 Overview**
This project is a **basic tutorial** for using a **Teensy 4.0** to control a **stepper motor** with an **A4988 driver** and a **limit switch** for homing. It is intended as a **starting point** for expanding into a more complex system that integrates **LiDAR sensors** for dynamic movement control.

## **📂 Project Structure**
```
joseriyancyriac-basic-teensy-tutorial/
├── platformio.ini   # PlatformIO configuration for Teensy 4.0
├── include/         # Project headers (currently unused)
├── lib/             # Private libraries (if needed later)
├── src/
│   └── main.cpp     # Main firmware code
└── test/            # Test files for unit testing
```

## **⚙️ Features Implemented**
✅ **Stepper Motor Control** – Uses the **AccelStepper** library.  
✅ **Enable Pin Logic** – **Stops/Disables motor** when necessary.  
✅ **Gravity Compensation** – **Motor stays active** to prevent freefall when needed.  
✅ **Limit Switch Integration** – Placed at the **topmost position** to prevent over-travel.  
✅ **Homing Routine** – Moves to **limit switch, backs off 10 steps**, sets **zero position**.  
✅ **Manual Control** – **W/S keys** allow manual movement **up/down**.  

## **🎯 Next Steps**
🔹 **Integrate LiDAR Sensor Data** for automatic positioning.  
🔹 **Optimize Movement Logic** for real-world application.  

## **💾 Commands**
- **`h`** → Home the stepper motor (move to limit switch).  
- **`w`** → Move **up** to the **home position (0 steps)**.  
- **`s`** → Move **down** to the **max displacement (-588 steps)**.  
- **`d`** → **Stop motor** and disable driver.  

## **🛠️ Dependencies**
- **Teensy 4.0**  
- **A4988 Stepper Driver**  
- **NEMA 17 Stepper Motor**  
- **AccelStepper Library** (`waspinator/AccelStepper@^1.64`)  

---

📌 **This is a foundational project. The next step is integrating LiDAR sensor feedback.** 🚀  
📂 **Repo maintained for future reference.**  

---
