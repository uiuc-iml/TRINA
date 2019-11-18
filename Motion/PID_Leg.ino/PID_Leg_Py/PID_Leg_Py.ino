//potentiometer min = 0 && max = 234
//-----------Pin No.-------------------------------------
const int leg_pwm = 3; //pin number_leg_pwm
const int leg_dir = 4; //pin mumber leg_direction
const int leg_potPin = A0; //potentiometer feedback read pin
//-----------Changing Values -----------------------------
int leg_potVal = 0; //potentiometer value
double current_pos = leg_potVal / 125; // 1inch = leg_potVal of 125 
boolean ext = false;
boolean ret = false;
//-----------PID------------------------------------------
//param
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
float potPosition_Float = 0;// input from tilt encoder
//float loc_calibration = 140; //value = loc input - actual loc

//-----------------------------------------------------------
 void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(leg_pwm, OUTPUT);
  pinMode(leg_dir, OUTPUT);
  pinMode(5, OUTPUT);
  digitalWrite(leg_dir,LOW);
}
void loop() {

  send_message(leg_current_loc, moving); //send current state to python
  //send_message(leg_target, moving);
  int good_message = poll_message(leg_target);  
  leg_current_loc = (float)(analogRead(leg_potPin) / (117.0));

  if(good_message == 0){
    // check if this is a special "handshake" message from the python side
    if (leg_target == 0xDEAD){
    send_message(0xFACE, moving);
    reachedTarget = false;
    return;
  }

  leg_pidCalc(leg_current_loc, leg_target);
  //runMotor(2- leg_current_loc);
  // target = 2*sin(0.5*currentTime);
  // delay(10);
  // currentTime += 0.01;
  }
  delay(10);
}


void runMotorTest()
{
  digitalWrite(leg_dir,LOW);
  digitalWrite(leg_pwm,HIGH);
  delayMicroseconds(100);
  digitalWrite(leg_pwm,LOW);
  delayMicroseconds(100);
  
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

void home(double cur_pos){
  if(cur_pos > 0){
    digitalWrite(leg_dir, HIGH);
    analogWrite(leg_pwm, 235);
  }
}

float mapValues(double value, double fromLow, double fromHigh, double toLow, double toHigh){
    
    float f = (value-fromLow) * (toHigh - toLow) / (float)((fromHigh - fromLow) + toLow);
    return f;
}

void extendMaximum(double cur_pos){
  if(cur_pos < 2){
    digitalWrite(leg_dir, LOW);
    analogWrite(leg_pwm, 235);
  }
}

void send_message(double current_pos, bool moving){
  // serialized data format: "TRINA\t[loc]\tTRINA\n"
  Serial.print("TRINA\t");
  Serial.print(current_pos);
  Serial.print("\t");
  Serial.print(moving);
  Serial.println("\tTRINA");
}

int poll_message(double &leg_target){
  if (!(Serial.available() > 0)){
    return -1;
  }
  int N = 100;
  char buf[N];
  int rv = 0;
  
  String header_str = Serial.readStringUntil('\t');
  String position_str = Serial.readStringUntil('\t');
  String footer_str = Serial.readStringUntil('\t');
  
  if (header_str != "TRINA"){
    rv = 1;
  }
  if (footer_str != "TRINA"){
    rv = 2;
  }

  if (rv != 0){
    return rv;
  }
  
  position_str.toCharArray(buf, N);
  leg_target = atof(buf);

  return 0;
}
