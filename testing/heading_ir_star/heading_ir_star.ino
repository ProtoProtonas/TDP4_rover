// main loop timing
#define main_loop_frequency 250  // in Herz
long int now = 0;
long int next_time = 0;
long int started = 0;
int period = 1000/main_loop_frequency; // in milliseconds
unsigned int up_counter = 0;


// star detection
#define ir_star_led1 28
#define ir_star_led2 29
#define ir_star_led3 30
#define ir_star_led4 31
#define ir_star_led5 32
#define ir_star_led6 33
#define ir_star_led7 34
#define ir_star_led8 35


// star detection variables (infrared sensors)
bool star1 = false;
bool star2 = false;
bool star3 = false;
bool star4 = false;
bool star5 = false;
bool star6 = false;
bool star7 = false;
bool star8 = false;
short int heading = 0; // in degrees
short int star_sum = 0;


void setup() {
  // put your setup code here, to run once:
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
  // put your main code here, to run repeatedly:

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

  Serial.print("Heading is ");
  Serial.print(heading);
  Serial.println(" degrees");


  
  now = millis();
  while(now < next_time) {
    now = millis();
  }
  up_counter++;
}
