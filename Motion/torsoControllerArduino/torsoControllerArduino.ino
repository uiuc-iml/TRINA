#include <Servo.h> 
Servo myservo;

//Program setup
int initial_loop_count = 0;
double loop_time = millis();
double previous_loop_time = 0;
double loop_frequency = 0;

//Height Potentiometer Setup (String potentiometer)
#define Height_PotPin    A2     // Analog pin 2 for the potentiometer

//Height Motor Setup
#define dtPulseWidth     4600  //Time between pulses: 10ms (2.9~100ms)
#define Height_PWMPin    6      //Digital pin 6 for motor driver
#define baudRate        115200     //Need faster baudrate to prevent miss counting turns
bool stop_height = false;

//Tilt Motor Setup
#define Tilt_PWMPin_1  3  //PWM
#define Tilt_PWMPin_2  13 //direction
double speed = 20;
int loop1 = 0;
bool stop_tilt = false;

//Support Leg Setup
const int leg_potPin = A0;
#define leg_dir 4 //DIR
#define leg_pwm 3 //PWM
double leg_kp = 0.355, leg_ki = 0.05, leg_kd = 0.0042; 
double leg_error, leg_de, leg_ie; //param
double leg_error_last;
double dt = 0.01;
double currentTime = 0.0;
double previous_time = micros(); //previous time var. Default set to current time
int dtpulsewidth = 1000;
double leg_target = 0.3; //******************************************************************************
boolean reachedTarget=false;
//-----------------------Python Comm Setup-------------------
double leg_target_max = 0;
double leg_target_min = 2;
double leg_error_range = 0.01;
double leg_current_loc = 0;
bool moving = false;

//Tilt Potentiometer Setup (AMT23 Encoder)
//Define special ascii characters
#define carriageReturn  0x0D
#define newLine         0x0A
#define tab             0x09
#define NOP __asm__ __volatile__ ("nop\n\t")
#define SSI_CS          7  //SSI Chip Select: Pin 7
#define SSI_SCK         10 //SSI Clock (SCK): Pin 10
#define SSI_SDO         11 //SSI Data (SDO):  Pin 11
#define res14           14
#define res12           12
uint16_t encoderPosition; //holder for encoder position
uint8_t attempts; //we can use this for making sure position is valid
double encoderPosition_Float = 0;
double encoderPosition_Float_Old = 0;
double turn_count = 0.0;
int read_count = 0;
/* 
 * AMT23 Pin Connections
 * Vdd (5V):              Pin 1
 * SSI DATA (SDO):        Pin 2
 * SSI CLOCK (SCK):       Pin 3
 * GND:                   Pin 4
 * Mode (unconnnected:    Pin 5
 * SSI Chip Select:       Pin 6
*/

//Python Communication Setup
//For Height
#define target_height_max    0.47 //0.47
#define target_height_min    0.05
#define height_error_range   0.01
#define height_error_coeff   3
double target_height = 0.25; //set target height here between 0.05 and 0.47
double previous_target_height = 0.3;
double current_height = 0;
bool lift_moving = false;
bool lift_goal_reached = true;
//For Tilt
#define target_tilt_max    265 //Lowest point 
#define target_tilt_min    235 //Highest point (perpendicular)
#define tilt_error_range   1   //unit: degrees
#define tilt_error_coeff   3
#define lowest_tilt_pwm    20  //lowest PWM signal
double target_tilt = 265;
double previous_target_tilt = 250;
double current_tilt = 0;
bool tilt_moving = true;
bool tilt_goal_reached = true;

//Height PID setup
#define height_kp   80 
#define height_ki   5
#define height_kd   0             // PID parameters for height
double height_e_last, dt_height;
double previous_time_height = millis(); //initialize previous time to current time

//Tilt PID setup
#define tilt_kp   10  
#define tilt_ki   0
#define tilt_kd   0             // PID parameters for tilt
double tilt_e_last, dt_tilt;
double previous_time_tilt = millis(); //initialize previous time to current time


int count = 0;

void setup()
{
  Serial.begin(baudRate);
  myservo.attach(Height_PWMPin);
  //pinMode(Height_PWMPin, OUTPUT);
  pinMode(Height_PotPin, INPUT);
  pinMode(Tilt_PWMPin_1, OUTPUT);
  pinMode(Tilt_PWMPin_2, OUTPUT);
  pinMode(SSI_SCK, OUTPUT);
  pinMode(SSI_CS, OUTPUT);
  pinMode(SSI_SDO, INPUT);
  digitalWrite(Tilt_PWMPin_1, LOW);
  digitalWrite(Tilt_PWMPin_2, LOW);
}

void loop()
{  
  //loop_frequency_check(); //use this function to check thie Arduino loop frequency
  height_position_read();
  tilt_position_read();
  leg_current_loc = (float)(analogRead(leg_potPin) / (117.0));

  
  if(initial_loop_count == 0){ //make robot stay at it's initial position
    target_height = current_height;
    target_tilt = current_tilt;
    initial_loop_count++;
  }
  /*
  //for handshake
  int good_message = python_communication();
  if(good_message != 0){
    count++;
    if(count>10){
      stop_motor_height();
      stop_motor_tilt();   
    }
    return; //does not move motor unless receive good message
  }else{
    count = 0;
  }
  */
  height_validation_execution();
  tilt_validation_execution();
  leg_pidCalc(leg_current_loc, leg_target);
}

void loop_frequency_check(){
  previous_loop_time = loop_time;
  loop_time = millis();
  loop_frequency = 1/(loop_time-previous_loop_time); //can use printf() to print out Arduino's loop frequency
}

int python_communication(){
  int good_message = poll_message(target_height, target_tilt);//0 if succeeded, -1 if no serial, 1 for bad

  send_message(current_height, current_tilt, lift_moving, tilt_moving, lift_goal_reached, tilt_goal_reached); //send current state to python

  if (good_message != 0){
    stop_motor_height();
    stop_motor_tilt();
    return 1;
  }

  // check if this is a special "handshake" message from the python side
  if (target_height == 0xDEAD && target_tilt == 0xBEEF){
    send_message(0xFACE, 0xB00C, lift_moving, tilt_moving, lift_goal_reached, tilt_goal_reached);
    target_height = 0.25;
    target_tilt = 250;
    stop_motor_height();
    reachedTarget=false;
    stop_motor_tilt();
    return 1;
  }
  return 0;
}

void height_position_read(){
  current_height = analogRead(Height_PotPin) * 25.0 * 0.0254 / 1024.0;
}

void tilt_position_read(){
  attempts = 0; //set attemps counter at 0 so we can try again if we get bad position  
  encoderPosition = getPositionSSI_efficient(res14); 

  while (encoderPosition == 0xFFFF && attempts++ < 3)
  {
    delay(10); //important for reading correct data, 1ms doesn't work
    encoderPosition = getPositionSSI_efficient(res14); //try again
  }

  if(encoderPosition == 0xFFFF){
    encoderPosition_Float = encoderPosition_Float_Old;
  }

  encoderPosition_Float = encoderPosition*0.021974; //turn the encoder's value in degrees

  //if the encoder turns more than 1 loop
  /*
  if(read_count == 0 && encoderPosition == 0xFFFF){ //first time reading in values
  }else if(read_count == 0){//first time reading in valid values
    read_count++;
  }else{
    if(encoderPosition_Float_Old > (encoderPosition_Float+350.0)){// going from 360 to 1. add 1 turn
      turn_count++;
    }else if(encoderPosition_Float_Old < (encoderPosition_Float-350.0)){ // going from 1 to 360. subtract 1 turn
      turn_count--;
    }
  }
  */
  current_tilt = encoderPosition_Float + 360.0*turn_count; 
  encoderPosition_Float_Old = encoderPosition_Float; 
}

void height_validation_execution(){
  dt_height = millis() - previous_time_height;
  previous_time_height = millis();
  
  if(target_height >= target_height_min && target_height <= target_height_max && current_height >= target_height_min && current_height <= target_height_max ){ //check if the target height is within range
    if(abs(current_height-target_height)==0){
      stop_motor_height();
      stop_height = true;  
      lift_goal_reached = true;
    }else if(stop_height == false && abs(current_height-target_height)>0){
      height_pid_control(current_height, target_height); 
      previous_target_height = target_height;
      lift_goal_reached = false;
    }
    
    if(stop_height == true && (target_height != previous_target_height)){
      stop_height = false;
      lift_goal_reached = false;
    }
  }else{     
  }
  
}

void tilt_validation_execution(){
  dt_tilt = millis() - previous_time_tilt;
  previous_time_tilt = millis();
  
  if(target_tilt >= target_tilt_min && target_tilt <= target_tilt_max && current_tilt >= target_tilt_min && current_tilt <= target_tilt_max ){ //check if the target tilt is within range
   if(abs(current_tilt-target_tilt)==0){
      stop_motor_tilt();
      stop_tilt = true;  
      tilt_goal_reached = true;
    }else if(stop_tilt == false && abs(current_tilt-target_tilt)>0){
      tilt_pid_control(current_tilt, target_tilt); 
      previous_target_tilt = target_tilt;
      tilt_goal_reached = false;
    }

    if(stop_tilt == true && (target_tilt != previous_target_tilt)){
      stop_tilt = false;
      tilt_goal_reached = false;
    }    
  } 
}

float height_pid_control(double current_height, double target_height) {
  double e, de, ie;
  e = target_height - current_height;
  de = (e - height_e_last) / dt_height;
  ie = ie + e * dt_height;
  height_e_last = e;
  double u = height_kp * e + height_ki * ie + height_kd * de;  
  run_motor_height(u); //power the motor
  return 0;
}

float tilt_pid_control(double current_tilt, double target_tilt) { 
  double e, de, ie;
  e = target_tilt - current_tilt;
  de = (e - tilt_e_last) / dt_tilt;
  ie = ie + e * dt_tilt;
  tilt_e_last = e;
  double u = tilt_kp * e + tilt_ki * ie + tilt_kd * de;
  run_motor_tilt(u);
  return 0;
}

void run_motor_height(double u) {
  lift_moving = true; //shows that the lift motor is moving
  if (u > 3)u = 3;
  if (u < -3)u = -3;
  double height_pwm = mapfloat(u, -3.0, 3.0, 1300, 1700); //1000us = clockwise, 2000us = counter-clockwise, map [-1000, 1000] to [1000, 2000]
  int height_pwm_int = (int)height_pwm;
  myservo.writeMicroseconds(height_pwm_int);
  
  /*
  digitalWrite(Height_PWMPin, HIGH);
  delayMicroseconds(height_pwm_int); //Pulse width: 2ms (0.6~2.4ms)
  digitalWrite(Height_PWMPin, LOW);
  delayMicroseconds(dtPulseWidth - height_pwm_int);
  */
}

void run_motor_tilt(double u) {
  tilt_moving = true; //shows that the lift motor is moving
  
  if (u > 100) { //cap value of u
    u = 100;
  } else if (u < -100) {
    u = -100;
  }
  
  double tilt_pwm = mapfloat(u, -100, 100, -255, 255); //1000us = clockwise, 2000us = counter-clockwise, map [-1000, 1000] to [1000, 2000]
  if(tilt_pwm <= lowest_tilt_pwm && tilt_pwm >0){ //cap lowest speed
    tilt_pwm = lowest_tilt_pwm;
  }else if(tilt_pwm >= -lowest_tilt_pwm && tilt_pwm < 0){
    tilt_pwm = -lowest_tilt_pwm;    
  }
  
  // Set the speed and direction.
  if (tilt_pwm <= 0) { //go up
    analogWrite(Tilt_PWMPin_1, -tilt_pwm);
    digitalWrite(Tilt_PWMPin_2, HIGH);
  } else { //go down
    analogWrite(Tilt_PWMPin_1, tilt_pwm);
    digitalWrite(Tilt_PWMPin_2, LOW);
  }
}

void stop_motor_height(){
  myservo.writeMicroseconds(1500);
  lift_moving = false; //shows that the lift motor is not moving
  /*
  digitalWrite(Height_PWMPin, HIGH);
  delayMicroseconds(1500); //Pulse width: 1.5ms to stop
  digitalWrite(Height_PWMPin, LOW);
  delayMicroseconds(dtPulseWidth - 1500);
  */
}

void stop_motor_tilt(){
  tilt_moving = false; //shows that the lift motor is not moving
  analogWrite(Tilt_PWMPin_1, 0);
  digitalWrite(Tilt_PWMPin_2, HIGH);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

uint16_t getPositionSSI_efficient(uint8_t resolution)
{
  uint8_t i, j; //we'll use these incrementers
  uint8_t odd, even; //bit parity counters
  uint16_t currentPosition = 0;
  uint8_t _clockCounts = resolution + 2; //the AMT23 includes 2 additional bits in the response that are used as checkbits
  uint8_t checkBit1, checkBit0; //the frist two bits in the position response are checkbits used to check the validity of the position response

  //drop cs low and wait the minimum required time. This is done with NOPs
  digitalWrite(SSI_CS, LOW);
  for (i = 0; i < 5; i++) NOP;

  //We will clock the encoder the number of times (resolution + 2), incrementing with 'j'
  //note that this method of bit-banging doesn't give a reliable clock speed.
  //in applications where datarate is important, the Arduino is not the best solution unless you
  //can find a way to make the SPI interface work for this protocol format.
  for (j = 0; j < _clockCounts; j++)
  {
    //first we lower the clock line and wait until the pin state has fully changed
    digitalWrite(SSI_SCK, LOW);
    for (i = 0; i < 10; i++) NOP;

    //now we go high with the clock. no need to wait with NOPs because the pin read we'll do next times sufficient time
    digitalWrite(SSI_SCK, HIGH);
    
    //throw the pin value into the position, note that it's reversing it as well
    currentPosition |= (digitalRead(SSI_SDO) << (_clockCounts - j - 1));
  }
  //release cs line, position has been fully received
  digitalWrite(SSI_CS, HIGH);


  //grab the highest two bits and put them into the checkbit holders
  checkBit1 = (currentPosition >> (_clockCounts - 1)) & 0x01;
  checkBit0 = (currentPosition >> (_clockCounts - 2)) & 0x01;

  //at this point currentPosition still holds the checkbits. So if we're in 14 bit mode, there's 16 bits
  //we only move up 14 bits here (resolution) because we're only tallying up the 1's in the position
  //we're counting the bits in even slots and the ones in odd slots
  for (uint8_t i = 0; i < resolution; i++) (i % 2 == 0) ? even += ((currentPosition >> i) & 0x01) : odd += ((currentPosition >> i) & 0x01);

  //check the counts against the checkbits
  if ((checkBit1 == odd % 2) || (checkBit0 == even % 2)) currentPosition = 0xFFFF;
  else 
  {
    //this isn't the 'fastest' since we're introducting an if/else but doing
    // currentPosition &= 2^resolution; doesn't work because arduino has a problem with
    // powers.
    if (resolution == res12) currentPosition &= 0xFFF;
    else if (resolution == res14) currentPosition &= 0x3FFF;
  }

  return currentPosition;
}

float leg_pidCalc(double current_pos, double leg_target){
  //dt = micros() - previous_time;
  //previous_time = micros(); //reset the previous time
  leg_error = leg_target - current_pos; 
  leg_de = (leg_error - leg_error_last) / dt;
  leg_ie = leg_ie + leg_error * dt;
  leg_error_last = leg_error;
  double leg_pid = (leg_kp*leg_error) + (leg_ki * leg_ie) + (leg_kd * leg_de);
  if(reachedTarget){
     leg_runMotor(0);
    }else{
  leg_runMotor(leg_pid);
    }
  return 0;
}


void stopActuator(){
  digitalWrite(leg_dir, LOW);
  analogWrite(leg_pwm, 0);
}

void leg_runMotor(double leg_pid){
  if(leg_pid > 2){
    leg_pid = 2;   
 }if(leg_pid < -2){
  leg_pid = -2;
 }
 if(fabs(leg_pid) < (1.0/117.0)){
   digitalWrite(leg_dir, LOW);
   analogWrite(leg_pwm, 0);
   reachedTarget=true;       
   return;
    }else{

  double leg_pwm_double =  mapValues(fabs(leg_pid), 0, 2.0, 0, 255.0);
  int leg_pwm_int = (int)leg_pwm_double;
  
    if(leg_pid < 0){
       digitalWrite(leg_dir, HIGH);
       analogWrite(leg_pwm, 255);
       analogWrite(5, 255);
       //delayMicroseconds(leg_pwm_int);


      }else {
        digitalWrite(leg_dir, LOW);
        analogWrite(leg_pwm, 255);
        analogWrite(5, 255);
        //delayMicroseconds(leg_pwm_int);

    }
 Serial.print(leg_pwm_int);    
  }
  
}


float mapValues(double value, double fromLow, double fromHigh, double toLow, double toHigh){
    
    float f = (value-fromLow) * (toHigh - toLow) / (float)((fromHigh - fromLow) + toLow);
    return f;
}




void send_message(double height, double tilt, bool lift_moving, bool tilt_moving, bool lift_goal_reached, bool tilt_goal_reached){
  // serialized data format: "TRINA\t[tilt]\t[height]\t[tilt_limit_switch]\t[lift_limit_switch]\tTRINA\n"
  
  Serial.print("TRINA\t");
  Serial.print(tilt);
  Serial.print("\t");
  Serial.print(height);
  Serial.print("\t");
  Serial.print(tilt_moving);
  Serial.print("\t");
  Serial.print(lift_moving);
  /*Serial.print("\t");
  Serial.print(tilt_goal_reached);
  Serial.print("\t");
  Serial.print(lift_goal_reached);
  */
  Serial.println("\tTRINA");
  
}

int poll_message(double &target_height, double &target_tilt){
  if (!(Serial.available() > 0)){
    return -1;
  }
  int N = 100;
  char height_buf[N], tilt_buf[N];
  
  String header_str = Serial.readStringUntil('\t');
  if (header_str != "TRINA"){
    return 1;
  }
  String height_str = Serial.readStringUntil('\t');
  String tilt_str = Serial.readStringUntil('\t');
  String footer_str = Serial.readStringUntil('\t');
  if (footer_str != "TRINA"){
    return 1;
  }
  
  height_str.toCharArray(height_buf, N);
  target_height = atof(height_buf);
  tilt_str.toCharArray(tilt_buf, N);
  target_tilt = (atof(tilt_buf));
  
  return 0;
}
