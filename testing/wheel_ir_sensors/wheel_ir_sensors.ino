// main loop timing

#define high 14

#define main_loop_frequency 100  // in Hertz
long int now = 0;
long int next_time = 0;
long int started = 0;
int period = 1000/main_loop_frequency; // in milliseconds
unsigned int up_counter = 0;

// motor angle (IR ON/OFF pairs)
#define motor_ir_led1 12

// motor speed variables - infrared ON/OFF switches
bool last_pos1 = false;     // last position of the switch
int last_change_cycle1 = 0; // cycle number when the last position change was
short int motor_speed1 = 0; // speed in milliradians/s
int cycles = 0;



void setup() {

  Serial.begin(115200);
  // ############# SETUP MOTOR SPEED DETECTION #############
  pinMode(motor_ir_led1, INPUT);

  pinMode(high, OUTPUT);

}

void loop() {
  digitalWrite(high, HIGH);
  // put your main code here, to run repeatedly:
  started = millis();
  next_time = started + period;
  // wheel 1
  
  cycles = 0;
  if(digitalRead(motor_ir_led1) == 1) {
    if(last_pos1 == 0) {
      cycles = up_counter - last_change_cycle1;
      Serial.print("Motor speed: ");
      Serial.print(6283 / cycles / period);
      Serial.println(" mrad/s");
      last_pos1 = 1;
      last_change_cycle1 = up_counter;
    }
//    Serial.println("1");
  } else {
    if(last_pos1 == 1) {
      cycles = up_counter - last_change_cycle1;
      Serial.print("Motor speed: ");
      Serial.print(6283 / cycles / period);
      Serial.println(" mrad/s");
      last_pos1 = 0;
      last_change_cycle1 = up_counter;
    }
//    Serial.println("0");
  }

  
  now = millis();
  while(now < next_time) {
    now = millis();
  }
  up_counter++;
}
