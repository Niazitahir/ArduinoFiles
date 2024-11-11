#include <Wire.h>
 
char RxBuffer[100] = {0};
int RxIdx = 0;
 
void setup() {
  Wire.begin();  // Initialize I2C (Master Mode: address is optional)
}
 
void loop()
{
  Serial.begin(9600);
  delay(10);
    Wire.requestFrom(0x0F, 1);  // Request 6 bytes from slave device @ address 0x55
    // Slave may send less than requested
    while(Wire.available()) {
        RxBuffer[RxIdx++] = (char) Wire.read();  // Receive a byte & push it into the RxBuffer
    }
    delay(100);
    Serial.println(RxBuffer);
    Serial.println("Done");
    delay(100);
}