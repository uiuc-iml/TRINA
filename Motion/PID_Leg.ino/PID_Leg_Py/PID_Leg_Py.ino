//potentiometer min = 0 && max = 234
//-----------Pin No.-------------------------------------
const int pwm = 3; //pin number_PWM
const int dir = 4; //pin mumber direction
const int potPin = A0; //potentiometer feedback read pin
//-----------Changing Values -----------------------------
int potVal = 0; //potentiometer value
double current_pos = potVal / 125; // 1inch = potVal of 125 
boolean ext = false;
boolean ret = false;
//-----------PID------------------------------------------
//param
double kp = 0.355, ki = 0.05, kd = 0.0042; 
double error, de, ie; //param
double error_last;
double dt = 0.01;
double currentTime = 0.0;
double previous_time = micros(); //previous time var. Default set to current time
int dtpulsewidth = 1000;
double target = 1.0; //******************************************************************************
boolean reachedTarget=false;
//-----------------------Python Comm Setup-------------------
double target_max = 0;
double target_min = 2;
double error_range = 0.01;
double current_loc = 0;
bool moving = false;
float potPosition_Float = 0;// input from tilt encoder
//float loc_calibration = 140; //value = loc input - actual loc

//-----------------------------------------------------------
 void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(pwm, OUTPUT);
  pinMode(dir, OUTPUT);
  pinMode(5, OUTPUT);
  digitalWrite(dir,LOW);
}
void loop() {
  
  send_message(current_loc, moving); //send current state to python
  //send_message(target, moving);
  int good_message = poll_message(target);
  
  // check if this is a special "handshake" message from the python side
  if (target == 0xDEAD){
    send_message(0xFACE, moving);
    target = 0;
    stopActuator();
    return;
}
  
  current_loc = (float)(analogRead(potPin) / (117.0));
  pidCalc(current_loc, target);
  //runMotor(2- current_loc);
 // target = 2*sin(0.5*currentTime);
 // delay(10);
 // currentTime += 0.01;
}

void runMotorTest()
{
  digitalWrite(dir,LOW);
  digitalWrite(pwm,HIGH);
  delayMicroseconds(100);
  digitalWrite(pwm,LOW);
  delayMicroseconds(100);
  
}
float pidCalc(double current_pos, double target){
  //dt = micros() - previous_time;
  //previous_time = micros(); //reset the previous time
  error = target - current_pos; 
  de = (error - error_last) / dt;
  ie = ie + error * dt;
  error_last = error;
  double pid = (kp*error) + (ki * ie) + (kd * de);
  if(reachedTarget){
     runMotor(0);
    }else{
  runMotor(pid);
    }
  return 0;
}

void stopActuator(){
  digitalWrite(dir, LOW);
  analogWrite(pwm, 0);
}

void runMotor(double pid){
  if(pid > 2){
    pid = 2;   
 }if(pid < -2){
  pid = -2;
 }
 if(fabs(pid) < (1.0/117.0)){
   digitalWrite(dir, LOW);
   analogWrite(pwm, 0);
   reachedTarget=true;       
   return;
    }else{

  double pwm_double =  mapValues(fabs(pid), 0, 2.0, 0, 255.0);
  int pwm_int = (int)pwm_double;
  
    if(pid < 0){
       digitalWrite(dir, HIGH);
       analogWrite(pwm, 255);
       analogWrite(5, 255);
       //delayMicroseconds(pwm_int);


      }else {
        digitalWrite(dir, LOW);
        analogWrite(pwm, 255);
        analogWrite(5, 255);
        //delayMicroseconds(pwm_int);

    }
  }
}

void home(double cur_pos){
  if(cur_pos > 0){
    digitalWrite(dir, HIGH);
    analogWrite(pwm, 235);
  }
}

float mapValues(double value, double fromLow, double fromHigh, double toLow, double toHigh){
    
    float f = (value-fromLow) * (toHigh - toLow) / (float)((fromHigh - fromLow) + toLow);
    return f;
}

void extendMaximum(double cur_pos){
  if(cur_pos < 2){
    digitalWrite(dir, LOW);
    analogWrite(pwm, 235);
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

int poll_message(double &target){
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
  target = atof(buf);

  return 0;
}
