#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

#define baudrate 9600

int value_for_led = 0;
int value_to_pc = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(baudrate);
  SerialBT.begin("damn boi");
}

void loop() {
  // UART communication between ESP32 and Arduino here
  while(Serial.available() > 0) {
    value_to_pc = Serial.read();
    SerialBT.println(value_to_pc);
  }
  
  // Bluetooth communication between PC and ESP32 here
  while(SerialBT.available() > 0) {
    value_for_led = SerialBT.read();
    if(value_for_led != 10) {
      Serial.write(value_for_led);
    }
  }
  delay(50);
}
