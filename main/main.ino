// Libraries and stuff
//#include "Arduino.h"

#define onboard_led 13

/*

rover as seen from above:


     3       2       1
    ___     ___     ___
   |___|   |___|   |___| 
  ________________________
 | 3         2         1  | 
 |                        | 
 | 4                   8  | >>> driving direction
 |                        | 
 |_5_________6_________7__|
    ___     ___     ___
   |___|   |___|   |___|
    
     4       5       6
 
*/

// main loop timing
#define main_loop_frequency 250  // in Herz
long int now = 0;
long int next_time = 0;
long int started = 0;
int period = 1000/main_loop_frequency; // in milliseconds
unsigned int up_counter = 0;


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

// ###################################
// ##########  SENSOR PINS ###########
// ###################################

// motor voltage
#define motor_voltage1_pin 10
#define motor_voltage2_pin 11
#define motor_voltage3_pin 12
#define motor_voltage4_pin 13
#define motor_voltage5_pin 14
#define motor_voltage6_pin 15

// wheel angle (hall effect sensors)
#define wheel_hall_effect1 16
#define wheel_hall_effect2 17
#define wheel_hall_effect3 18
#define wheel_hall_effect4 19
#define wheel_hall_effect5 20
#define wheel_hall_effect6 21

// motor speed (IR ON/OFF pairs)
#define motor_ir_led1 22
#define motor_ir_led2 23
#define motor_ir_led3 24
#define motor_ir_led4 25
#define motor_ir_led5 26
#define motor_ir_led6 27

// star detection
#define ir_star_led1 28
#define ir_star_led2 29
#define ir_star_led3 30
#define ir_star_led4 31
#define ir_star_led5 32
#define ir_star_led6 33
#define ir_star_led7 34
#define ir_star_led8 35

// ultrasonic sensor pins


// ###################################
// #######  SENSOR VARIABLES  ########
// ###################################

// current variables - ADCs - I2C
// values stored mean current in milliamps, can measure up to 32767mA = 32.7A. If want to measure more - change type from short int to int
short int current1 = 0; // in mA
short int current2 = 0;
short int current3 = 0;
short int current4 = 0;
short int current5 = 0;
short int current6 = 0;


// voltage variables - onboard ADC
// values stored mean voltage in millivolts, can measure up to 32767mV = 32.7V. If want to measure more - change type from short int to int
short int voltage1 = 0; // in mV
short int voltage2 = 0;
short int voltage3 = 0;
short int voltage4 = 0;
short int voltage5 = 0;
short int voltage6 = 0;


// wheel angle/speed variables - Hall effect sensors.
short int wheel_angle1 = 0; // in degrees
short int wheel_speed1 = 0; // in mm/s
short int wheel_angle2 = 0;
short int wheel_speed2 = 0;
short int wheel_angle3 = 0;
short int wheel_speed3 = 0;
short int wheel_angle4 = 0;
short int wheel_speed4 = 0;
short int wheel_angle5 = 0;
short int wheel_speed5 = 0;
short int wheel_angle6 = 0;
short int wheel_speed6 = 0;


// motor speed variables - infrared ON/OFF switches
bool last_pos1 = false;     // last position of the switch
int last_change_cycle1 = 0; // cycle number when the last position change was
short int motor_speed1 = 0; // speed in milliradians/s
bool last_pos2 = false;
int last_change_cycle2 = 0;
short int motor_speed2 = 0;
bool last_pos3 = false;
int last_change_cycle3 = 0;
short int motor_speed3 = 0;
bool last_pos4 = false;
int last_change_cycle4 = 0;
short int motor_speed4 = 0;
bool last_pos5 = false;
int last_change_cycle5 = 0;
short int motor_speed5 = 0;
bool last_pos6 = false;
int last_change_cycle6 = 0;
short int motor_speed6 = 0;
int cycles = 0;


// imu variables - BNO055
int imu_angle_x = 0; // in degrees
int imu_angle_y = 0;
int imu_angle_z = 0;
int imu_accel_x = 0;
int imu_accel_y = 0;
int imu_accel_z = 0;


// star detection variables (infrared sensors)
bool star1 = false;
bool star2 = false;
bool star3 = false;
bool star4 = false;
bool star5 = false;
bool star6 = false;
bool star7 = false;
bool star8 = false;
short int heading = 0;
short int star_sum = 0;

// ultrasonic sensor variables
int distance_right = 0; // distance from an obstacle from the right ultrasonic sensor in mm
int distance_middle = 0;
int distance_left = 0;



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

  // ############# SETUP SERIAL PORTS ###############
  Serial.begin(115200); // for debugging, communicates with PC
  Serial1.begin(115200); // send data to ESP32

  // onboard LED to see how hard Arduino is working
  pinMode(onboard_led, OUTPUT); // the brighter the glow - the harder Atmega is working
  
  // ############# SETUP MOTOR OUTPUTS ##############
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
  digitalWrite(enable_motors, HIGH); // enable motors to be driven

  // ############# SETUP WHEEL SPEED DETECTION #############
  pinMode(wheel_hall_effect1, INPUT);
  pinMode(wheel_hall_effect2, INPUT);
  pinMode(wheel_hall_effect3, INPUT);
  pinMode(wheel_hall_effect4, INPUT);
  pinMode(wheel_hall_effect5, INPUT);
  pinMode(wheel_hall_effect6, INPUT);
  
  // ############# SETUP MOTOR SPEED DETECTION #############
  pinMode(motor_ir_led1, INPUT);
  pinMode(motor_ir_led2, INPUT);
  pinMode(motor_ir_led3, INPUT);
  pinMode(motor_ir_led4, INPUT);
  pinMode(motor_ir_led5, INPUT);
  pinMode(motor_ir_led6, INPUT);
  
  // ############# SETUP ULTRASONIC DETECTION #############
  // ############# SETUP MOTOR VOLTAGE DETECTION #############

  // ############# SETUP STAR DETECTION #############
  pinMode(ir_star_led1, INPUT);
  pinMode(ir_star_led2, INPUT);
  pinMode(ir_star_led3, INPUT);
  pinMode(ir_star_led4, INPUT);
  pinMode(ir_star_led5, INPUT);
  pinMode(ir_star_led6, INPUT);
  pinMode(ir_star_led7, INPUT);
  pinMode(ir_star_led8, INPUT);
  

}

void loop() {
  started = millis();
  next_time = started + period;
  // ###############################################################
  // ############# MOTOR SPEED DETECTION/CALCULATION ###############
  // ###############################################################
  
  // MOTOR 1
  cycles = 0;
  if(digitalRead(motor_ir_led1) == 1) {
    if(last_pos1 == 0) {
      cycles = up_counter - last_change_cycle1;
      motor_speed1 = 6283 / cycles / period; // in millirad/s
      last_pos1 = 1;
      last_change_cycle1 = up_counter;
    }
  } else {
    if(last_pos1 == 1) {
      cycles = up_counter - last_change_cycle1;
      motor_speed1 = 6283 / cycles / period; // in millirad/s
      last_pos1 = 0;
      last_change_cycle1 = up_counter;
    }
  }
  
  // MOTOR 2
  cycles = 0;
  if(digitalRead(motor_ir_led2) == 1) {
    if(last_pos2 == 0) {
      cycles = up_counter - last_change_cycle2;
      motor_speed2 = 6283 / cycles / period; // in millirad/s
      last_pos2 = 1;
      last_change_cycle2 = up_counter;
    }
  } else {
    if(last_pos2 == 1) {
      cycles = up_counter - last_change_cycle2;
      motor_speed2 = 6283 / cycles / period; // in millirad/s
      last_pos2 = 0;
      last_change_cycle2 = up_counter;
    }
  }
  
  // MOTOR 3
  cycles = 0;
  if(digitalRead(motor_ir_led3) == 1) {
    if(last_pos3 == 0) {
      cycles = up_counter - last_change_cycle3;
      motor_speed3 = 6283 / cycles / period; // in millirad/s
      last_pos3 = 1;
      last_change_cycle3 = up_counter;
    }
  } else {
    if(last_pos1 == 1) {
      cycles = up_counter - last_change_cycle3;
      motor_speed3 = 6283 / cycles / period; // in millirad/s
      last_pos3 = 0;
      last_change_cycle3 = up_counter;
    }
  }
  
  // MOTOR 4
  cycles = 0;
  if(digitalRead(motor_ir_led4) == 1) {
    if(last_pos4 == 0) {
      cycles = up_counter - last_change_cycle4;
      motor_speed4 = 6283 / cycles / period; // in millirad/s
      last_pos4 = 1;
      last_change_cycle4 = up_counter;
    }
  } else {
    if(last_pos4 == 1) {
      cycles = up_counter - last_change_cycle4;
      motor_speed4 = 6283 / cycles / period; // in millirad/s
      last_pos4 = 0;
      last_change_cycle4 = up_counter;
    }
  }
  
  // MOTOR 5
  cycles = 0;
  if(digitalRead(motor_ir_led5) == 1) {
    if(last_pos5 == 0) {
      cycles = up_counter - last_change_cycle5;
      motor_speed5 = 6283 / cycles / period; // in millirad/s
      last_pos5 = 1;
      last_change_cycle5 = up_counter;
    }
  } else {
    if(last_pos5 == 1) {
      cycles = up_counter - last_change_cycle5;
      motor_speed5 = 6283 / cycles / period; // in millirad/s
      last_pos5 = 0;
      last_change_cycle5 = up_counter;
    }
  }
  
  // MOTOR 6
  cycles = 0;
  if(digitalRead(motor_ir_led6) == 1) {
    if(last_pos6 == 0) {
      cycles = up_counter - last_change_cycle6;
      motor_speed6 = 6283 / cycles / period; // in millirad/s
      last_pos6 = 1;
      last_change_cycle6 = up_counter;
    }
  } else {
    if(last_pos6 == 1) {
      cycles = up_counter - last_change_cycle6;
      motor_speed6 = 6283 / cycles / period; // in millirad/s
      last_pos6 = 0;
      last_change_cycle6 = up_counter;
    }
  }

  // ###############################################################
  // ############# WHEEL SPEED DETECTION/CALCULATION ###############
  // ###############################################################
  
  // ###############################################################
  // ############ MOTOR VOLTAGE DETECTION/CALCULATION ##############
  // ###############################################################
  
  // ###############################################################
  // ############ MOTOR CURRENT DETECTION/CALCULATION ##############
  // ###############################################################
  
  // ###############################################################
  // ############ STAR DETECTION/ HEADING CALCULATION ##############
  // ###############################################################

  star1 = digitalRead(ir_star_led1); star2 = digitalRead(ir_star_led2); star3 = digitalRead(ir_star_led3); star4 = digitalRead(ir_star_led4); 
  star5 = digitalRead(ir_star_led5); star6 = digitalRead(ir_star_led6); star7 = digitalRead(ir_star_led7); star8 = digitalRead(ir_star_led8);
  
  star_sum = star1 + star2 + star3 + star4 + star5 + star6 + star7 + star8;
  if (star_sum == 0) {
    heading = 0;
  } else {
    heading = (star1 * 45 + star2 * 90 + star3 * 135 + star4 * 180 + star5 * 225 + star6 * 270 + star7 * 315) / star_sum;
    if (heading > 180) {
      heading -= 360;
    }
  }
  
  // ###############################################################
  // ############ BLUETOOTH RSSI DETECTION/CALCULATION #############
  // ###############################################################
  
  // ###############################################################
  // ########## ULTRASONIC OBSTACLE DETECTION/CALCULATION ##########
  // ###############################################################
  



  
  now = millis();
  digitalWrite(onboard_led, HIGH); // turn off the onboard LED
  while(now < next_time) {
    now = millis();
  }
  digitalWrite(onboard_led, LOW); // turn on the onboard LED

  // TO DO: add variables to send over to ESP32
  Serial1.print(" "); // send data to ESP32
  up_counter++;
}
