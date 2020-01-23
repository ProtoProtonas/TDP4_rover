/*
     3       2       1
    ___     ___     ___
   |___|   |___|   |___| 
  ________________________
 |                        | 
 |                        | 
 |                        | >>> driving direction
 |                        | 
 |________________________|
    ___     ___     ___
   |___|   |___|   |___|
    
     4       5       6
 
*/

// main loop timing
#define main_loop_frequency 100  // in Herz


// ###################################
// ########  MOTOR CONTROL  ##########
// ###################################


#define full_speed_straight 0.6
#define full_speed_turn 0.6
#define motor_pwm_frequency 500

// Pins for driving the motors with PWM signal
// MUST BE PWM ENABLED PINS (the ones with "~" symbol on them)
#define pwm_motor1 1
#define pwm_motor2 2
#define pwm_motor3 3
#define pwm_motor4 4
#define pwm_motor5 5
#define pwm_motor6 6

// Pins for controlling motor spin direction
#define direction_right 7
#define direction_left 8

// Enable pins for each motor (possible to wire them all together and have 1 enable pin for all motors)
#define enable_motors 9

float slip1 = 0, slip2 = 0, slip3 = 0, slip4 = 0, slip5 = 0, slip6 = 0;

void forward() {
    digitalWrite(direction_left, LOW);
    digitalWrite(direction_right, LOW);
    
    analogWrite(pwm_motor1, full_speed_straight);
    analogWrite(pwm_motor2, full_speed_straight);
    analogWrite(pwm_motor3, full_speed_straight);
    analogWrite(pwm_motor4, full_speed_straight);
    analogWrite(pwm_motor5, full_speed_straight);
    analogWrite(pwm_motor6, full_speed_straight);

}

void turn_right(float variable_speed) {
    digitalWrite(direction_left, LOW);
    digitalWrite(direction_right, HIGH);

    analogWrite(pwm_motor1, full_speed_turn);
    analogWrite(pwm_motor2, full_speed_turn);
    analogWrite(pwm_motor3, full_speed_turn);
    analogWrite(pwm_motor4, full_speed_straight);
    analogWrite(pwm_motor5, full_speed_straight);
    analogWrite(pwm_motor6, full_speed_straight);
}

void turn_left(float variable_speed) {
    digitalWrite(direction_left, HIGH);
    digitalWrite(direction_right, LOW);

    analogWrite(pwm_motor1, full_speed_straight);
    analogWrite(pwm_motor2, full_speed_straight);
    analogWrite(pwm_motor3, full_speed_straight);
    analogWrite(pwm_motor4, full_speed_turn);
    analogWrite(pwm_motor5, full_speed_turn);
    analogWrite(pwm_motor6, full_speed_turn);
}

void stop() {

    digitalWrite(direction_left, HIGH);
    digitalWrite(direction_right, HIGH);

    analogWrite(pwm_motor1, full_speed_turn);
    analogWrite(pwm_motor2, full_speed_turn);
    analogWrite(pwm_motor3, full_speed_turn);
    analogWrite(pwm_motor4, full_speed_turn);
    analogWrite(pwm_motor5, full_speed_turn);
    analogWrite(pwm_motor6, full_speed_turn);
    
    delay(50);
    
    digitalWrite(direction_left, LOW);
    digitalWrite(direction_right, LOW);

    analogWrite(pwm_motor1, 0);
    analogWrite(pwm_motor2, 0);
    analogWrite(pwm_motor3, 0);
    analogWrite(pwm_motor4, 0);
    analogWrite(pwm_motor5, 0);
    analogWrite(pwm_motor6, 0);
    
}


void setup() {
  //apparently, Arduino PWM is done with analogWrite from PWM enabled pins automatically

  pinMode(pwm_motor1, OUTPUT);
  pinMode(pwm_motor2, OUTPUT);
  pinMode(pwm_motor3, OUTPUT);
  pinMode(pwm_motor4, OUTPUT);
  pinMode(pwm_motor5, OUTPUT);
  pinMode(pwm_motor6, OUTPUT);

  pinMode(direction_right, OUTPUT);
  pinMode(direction_left, OUTPUT);
  
  pinMode(enable_motors, OUTPUT);
  digitalWrite(enable_motors, HIGH);

}

void loop() {
  // put your main code here, to run repeatedly:

}
