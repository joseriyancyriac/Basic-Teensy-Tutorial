#include <Arduino.h>

// ==================== TF Mini LiDAR Configuration ====================
HardwareSerial &lidarSerial = Serial2;  // Use Serial2 (RX1=pin 7, TX1=pin 8)

// Packet structure constants
const uint8_t HEADER_BYTE = 0x59;
const uint8_t PACKET_SIZE = 9;

// Data storage
uint8_t lidarBuffer[PACKET_SIZE];
int16_t distance = -1;   // -1 = invalid measurement
int16_t strength = -1;
bool validData = false;

void setup() {
  Serial.begin(115200);    // USB serial for debugging
  lidarSerial.begin(115200);  // TF Mini default baud rate

  while (!Serial); // Wait for USB connection
  
  Serial.println("\nTF Mini LiDAR Test - Teensy 4.0");
  Serial.println("--------------------------------");
}

void processLidarData() {
  validData = false;
  
  if (lidarSerial.available() >= PACKET_SIZE) {
    // Read entire packet
    lidarSerial.readBytes(lidarBuffer, PACKET_SIZE);

    // Verify packet headers
    if (lidarBuffer[0] == HEADER_BYTE && lidarBuffer[1] == HEADER_BYTE) {
      // Calculate checksum
      uint16_t checksum = 0;
      for (uint8_t i=0; i<8; i++) {
        checksum += lidarBuffer[i];
      }

      // Validate checksum
      if ((checksum & 0xFF) == lidarBuffer[8]) {
        distance = lidarBuffer[2] | (lidarBuffer[3] << 8);
        strength = lidarBuffer[4] | (lidarBuffer[5] << 8);
        validData = true;
      }
    }
  }
}

void loop() {
  processLidarData();

  if (validData) {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print(" cm\t");
    
    Serial.print("Strength: ");
    Serial.print(strength);
    
    // Add quality indicator
    Serial.print("\t[");
    if (strength > 200) Serial.print("HIGH");
    else if (strength > 100) Serial.print("MED");
    else Serial.print("LOW");
    Serial.println("]");
  }

  delay(50); // Adjust polling rate as needed
}