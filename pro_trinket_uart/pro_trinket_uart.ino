//#include <Serial.h>

#define baudrate 9600
#define in A1
#define low A0
#define high A2
#define led 13

int pin_value = 0, value_for_led = 0;
int last_led_value = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(in, INPUT);
  pinMode(high, OUTPUT);
  pinMode(low, OUTPUT);
  digitalWrite(high, HIGH);
  digitalWrite(low, LOW);
  
  Serial.begin(baudrate);
}

void loop() {
  
  while(Serial.available() > 0) {
    value_for_led = Serial.read();
    if(value_for_led != 10){
      last_led_value = value_for_led;  
    }
  }

  if(last_led_value > 53) {
    digitalWrite(led, LOW);  
  } else {
    digitalWrite(led, HIGH);
  }
  
  

  pin_value = analogRead(in);
  Serial.write(pin_value);

  delay(200);
  
}
