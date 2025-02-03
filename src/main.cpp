#include <Arduino.h>

#include <Wire.h>               // SCL pin 19, SDA pin 18
#include <Adafruit_GFX.h>      
#include <Adafruit_SSD1306.h>   

Adafruit_SSD1306 display(128, 64, &Wire0, -1, 1000000);  // 1MHz I2C clock

void setup()
{
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  delay(100);
  display.clearDisplay();
  display.display();
}

void loop()
{
  display.clearDisplay(); // clear buffer
  for (int r = 0; r < 9; r++)
  {
    for (int y = 0; y < 8; y++) // fill buffer completely withs chars
    {
      display.setTextSize(1);
      display.setTextColor(1);
      display.setCursor(0, (y * 8));
      display.print("012345678901234567890");
    }
    display.setTextSize(1); // erase one line at the time
    display.setTextColor(0);
    display.setCursor(0, (r * 8));
    display.print("012345678901234567890");

    display.display();
    delay(200);
  }
}