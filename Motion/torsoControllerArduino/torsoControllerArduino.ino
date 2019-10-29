//for potentiometer
int Height_PotPin = A2;    // Analog pin 2 for the potentiometer
int val = 0;       // variable to store the value coming from the sensor

//Height Motor Setup
int dtPulseWidth = 10000;//Time between pulses: 10ms (2.9~100ms)
int Height_PWMPin = 6; //Digital pin 6 for motor driver

//Tilt Motor Setup
int Tilt_PWMPin = 5;

//Tilt Potentiometer Setup


//Python communication setup
double target_height_max = 0.2;
double target_height_min = 0.4;
double target_height = 0.25; //set target height here between 0.2 and 0.4
double target_tilt = 0;
double current_height = 0;
double current_tilt = 0;
bool lift_motion = false;
bool tilt_limit_switch = false;
float encoderPosition_Float = 0;// input from tilt encoder
float height_calibration = 140; //value = height input - actual height, 140 when at lowest point

//PID setup
double height_kp = 80 , height_ki = 5 , height_kd = 0;             // PID parameters for height
double height_e_last, tilt_e_last, dt;
double previous_time = millis(); //initialize previous time to current time

//Initial waittime
bool wait = false;

void setup()
{
  pinMode(Height_PWMPin, OUTPUT);
  pinMode(Height_PotPin, INPUT);
  Serial.begin(9600);
}
void loop()
{
  //Pulse width: 2ms (0.6~2.4ms), initial wait for 3 seconds
  if(wait == false){
    delay(3000); 
    wait = true;
  }

  //read in sensors values, receive target and send current states
  double current_height = analogRead(Height_PotPin) * 25.0 * 0.0254 / 1024.0;
  poll_message(target_height, target_tilt);
  //current_tilt = encoderPosition_Float;    // input from tilt encoder (add ANT23 code to this)
  send_message(current_height, current_tilt, lift_motion, tilt_limit_switch); //send current state to python
  
  dt = millis() - previous_time;
  previous_time = millis();
    
  if(target_height > target_height_min && target_height < target_height_max){ //check if the target height is within range
    if(abs(current_height-target_height)<0.01){
      stop_motor_lift();
    }else{
      height_pid_control(current_height, target_height);      
    }
  }  
}

float height_pid_control(double current_height, double target_height) {
  double e, de, ie;
  e = target_height - current_height;
  de = (e - height_e_last) / dt;
  ie = ie + e * dt;
  height_e_last = e;
  double u = height_kp * e + height_ki * ie + height_kd * de;

  /*
  double print_target = target_height*100;
  double print_current = current_height*100;
  //print to plotter
  Serial.print(print_target);
  Serial.print(" ");
  Serial.println(print_current);
  */
  
  run_motor_lift(u); //power the motor
  return 0;
}

/*
float tilt_pid_control(double current_tilt, double target_tilt) { 
  double e, de, ie;
  e = target_height - current_height;
  de = (e - height_e_last) / dt;
  ie = ie + e * dt;
  height_e_last = e;
  double u = height_kp * e + height_ki * ie + height_kd * de;
  
  double print_target = target_height*100;
  double print_current = current_height*100;
  run_motor_tilt(u);       
}
*/

void run_motor_lift(double u) {
  lift_motion = true; //shows that the lift motor is moving
  if (u > 3)u = 3;
  if (u < -3)u = -3;
  double height_pwm = mapfloat(u, -3.0, 3.0, 1400, 1600); //1000us = clockwise, 2000us = counter-clockwise, map [-1000, 1000] to [1000, 2000]
  int height_pwm_int = (int)height_pwm;
  digitalWrite(Height_PWMPin, HIGH);
  delayMicroseconds(height_pwm_int); //Pulse width: 2ms (0.6~2.4ms)
  digitalWrite(Height_PWMPin, LOW);
  delayMicroseconds(dtPulseWidth - height_pwm_int);
}

void stop_motor_lift(){
  lift_motion = false; //shows that the lift motor is not moving
  digitalWrite(Height_PWMPin, HIGH);
  delayMicroseconds(1500); //Pulse width: 1.5ms to stop
  digitalWrite(Height_PWMPin, LOW);
  delayMicroseconds(dtPulseWidth - height_pwm_int);
}

/*
void run_motor_tilt(double u) {
  if (u > 3)u = 3;
  if (u < -3)u = -3;
  double height_pwm = mapfloat(u, -3.0, 3.0, 1400, 1600); //1000us = clockwise, 2000us = counter-clockwise, map [-1000, 1000] to [1000, 2000]
  int height_pwm_int = (int)height_pwm;
  digitalWrite(Height_PWMPin, HIGH);
  delayMicroseconds(height_pwm_int); //Pulse width: 2ms (0.6~2.4ms)
  digitalWrite(Height_PWMPin, LOW);
  delayMicroseconds(dtPulseWidth - height_pwm_int);
}
 */

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

void send_message(double height, double tilt, bool lift_motion, bool tilt_limit_switch){
  // serialized data format: "TRINA\t[tilt]\t[height]\t[tilt_limit_switch]\t[lift_limit_switch]\tTRINA\n"
  Serial.print("TRINA\t");
  Serial.print(tilt);
  Serial.print("\t");
  Serial.print(height);
  Serial.print("\t");
  Serial.print(tilt_limit_switch);
  Serial.print("\t");
  Serial.print(lift_motion);
  Serial.println("\tTRINA");
}

int poll_message(double &target_height, double &target_tilt){
  if (!(Serial.available() > 0)){
    return -1;
  }
  int N = 100;
  char height_buf[N], tilt_buf[N];
  String height_str = Serial.readStringUntil('\0');
  height_str.toCharArray(height_buf, N);
  target_height = atof(height_buf);
  String tilt_str = Serial.readStringUntil('\0');
  tilt_str.toCharArray(tilt_buf, N);
  target_tilt = atof(tilt_buf);
  return 0;
}
