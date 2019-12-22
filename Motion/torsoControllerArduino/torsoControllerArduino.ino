#include <Servo.h>

class PIDController {
  private:
    double kP, kI, kD;
    double integral_error, prev_error;
    double target;
    double min_output, max_output;
    double tol;
    double curr_millis, prev_millis;
    bool has_target, at_target;

  public:
    PIDController(double kP, double kI, double kD);
    double set_tolerance(double tol);
    double update(double current, double target);
    bool done();
};

enum LegState {CLOSED, MOVING, OPEN};

Servo lift_motor;

//Program setup
int initial_loop_count = 0;

// Arduino Port numbers
#define kLiftPotPin         A2
#define kLiftPwmPin         6
#define kTiltPwmPin         3
#define kTiltDirectionPin   13
#define kLegPotPin          A0
#define kLegDirectionPin    4
#define kLegPwmPin          9
#define kBaudRate        115200     //Need faster kBaudRate to prevent miss counting turns

//Tilt Potentiometer Setup (AMT23 Encoder)
#define carriageReturn  0x0D
#define newLine         0x0A
#define tab             0x09
#define NOP __asm__ __volatile__ ("nop\n\t")
#define SSI_CS          7  //SSI Chip Select: Pin 7
#define SSI_SCK         10 //SSI Clock (SCK): Pin 10
#define SSI_SDO         11 //SSI Data (SDO):  Pin 11
#define res14           14
#define res12           12
uint16_t encoder_pos_int; //holder for encoder position
uint8_t attempts; //we can use this for making sure position is valid
double encoder_pos_float = 0;
double encoder_pos_float_old = 0;
double turn_count = 0.0;

// Lift constraints/status
#define target_height_max    0.47 //0.47
#define target_height_min    0.05
#define height_error_range   0.01
#define height_error_coeff   3
double target_height = 0.25; //set target height here between 0.05 and 0.47
double previous_target_height = 0.3;
double current_height = 0;
bool lift_moving = false;
bool lift_goal_reached = true;
bool stop_height = false;

//Tilt constraints/status
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
bool stop_tilt = false;

//Support Leg constraints/status
double target_leg = 0.35;  //minimum = 0.35
boolean leg_goal_reached = false;
#define target_leg_max 0
#define target_leg_min 2
#define leg_error_range 0.01
double leg_current_loc = 0;
#define leg_soft_limit 0.35
bool leg_moving = false;
LegState leg_state;

//Lift PID
#define height_kp   80
#define height_ki   5
#define height_kd   0
PIDController height_pid_controller(height_kp, height_ki, height_kd);

//Tilt PID
#define tilt_kp   10
#define tilt_ki   0.5
#define tilt_kd   0
PIDController tilt_pid_controller(tilt_kp, tilt_ki, tilt_kd);

//Support Leg PID
#define leg_kp    3.0
#define leg_ki    0.0
#define leg_kd    0.0042
PIDController leg_pid_controller(leg_kp, leg_ki, leg_kd);

//Python communication flag
int bad_message_count = 0;

void setup() {
  Serial.begin(kBaudRate);
  lift_motor.attach(kLiftPwmPin);
  pinMode(kLiftPotPin, INPUT);
  pinMode(kTiltPwmPin, OUTPUT);
  pinMode(kTiltDirectionPin, OUTPUT);
  pinMode(SSI_SCK, OUTPUT);
  pinMode(SSI_CS, OUTPUT);
  pinMode(SSI_SDO, INPUT);
  digitalWrite(kTiltPwmPin, LOW);
  digitalWrite(kTiltDirectionPin, LOW);
}

void loop() {
  height_position_read();
  tilt_position_read();
  leg_current_loc = (float)(analogRead(kLegPotPin) / (117.0));
  if ((2 - leg_current_loc) <= (1 / 117)) {
    leg_state = OPEN;
  } else if ((leg_current_loc - leg_soft_limit) <= (1 / 117)) {
    leg_state = CLOSED;
  } else leg_state = MOVING;


  if (initial_loop_count == 0) { //make robot stay at it's initial position
    target_height = current_height;
    target_tilt = current_tilt;
    target_leg = leg_current_loc;
    initial_loop_count++;
  }

  int rv = poll_message(target_height, target_tilt, target_leg);

  // check if this is a special "handshake" message from the python side
  if (rv == 1) {
    Serial.print("yo\n");
  } else if (rv == 0) {
    send_message(current_height, current_tilt, leg_state);
  } else {
    Serial.print("BAD\n");
  }

  if (rv != 0) {
    bad_message_count++;
    if (bad_message_count > 20) {
      stop_motor_height();
      stop_motor_tilt();
      stop_leg();
      return; //does not move motor unless receive good message
    }
  } else {
    bad_message_count = 0;
    height_validation_execution();
    tilt_validation_execution();
    if (target_leg <= leg_soft_limit) {
      target_leg = leg_soft_limit;
    }
    double u = leg_pid_controller.update(leg_current_loc, target_leg);
    run_leg(u);
  }

  delay(10);
}

void height_position_read() {
  current_height = analogRead(kLiftPotPin) * 25.0 * 0.0254 / 1024.0;
}

void tilt_position_read() {
  attempts = 0; //set attemps counter at 0 so we can try again if we get bad position
  encoder_pos_int = getPositionSSI_efficient(res14);

  while (encoder_pos_int == 0xFFFF && attempts++ < 3)
  {
    //Serial.print("");
    //delay(10); //important for reading correct data, 1ms doesn't work
    encoder_pos_int = getPositionSSI_efficient(res14); //try again
  }

  if (encoder_pos_int == 0xFFFF) {
    encoder_pos_float = encoder_pos_float_old;
  }

  encoder_pos_float = encoder_pos_int * 0.021974; //turn the encoder's value in degrees

  double val = encoder_pos_float + 360.0 * turn_count;
  if (val < 280) {
    current_tilt = val;
  }
  encoder_pos_float_old = encoder_pos_float;
}

void height_validation_execution() {
  if (target_height >= target_height_min && target_height <= target_height_max && current_height >= target_height_min && current_height <= target_height_max ) { //check if the target height is within range
    if (abs(current_height - target_height) < height_error_range) {
      stop_motor_height();
      stop_height = true;
      lift_goal_reached = true;
    } else if (stop_height == false && abs(current_height - target_height) >= height_error_range) {
      double u = height_pid_controller.update(current_height, target_height);
      run_lift(u);
      previous_target_height = target_height;
      lift_goal_reached = false;
    }

    if (stop_height == true && (target_height != previous_target_height)) {
      stop_height = false;
      lift_goal_reached = false;
    }
  }

}

void tilt_validation_execution() {

  if (target_tilt >= target_tilt_min && target_tilt <= target_tilt_max && current_tilt >= target_tilt_min && current_tilt <= target_tilt_max ) { //check if the target tilt is within range
    if (abs(current_tilt - target_tilt) < tilt_error_range) {
      stop_motor_tilt();
      stop_tilt = true;
      tilt_goal_reached = true;
    } else if (stop_tilt == false && abs(current_tilt - target_tilt) >= tilt_error_range) {
      double u = tilt_pid_controller.update(current_tilt, target_tilt); //tilt_pid_control(current_tilt, target_tilt);
      run_tilt(u);
      previous_target_tilt = target_tilt;
      tilt_goal_reached = false;
    }

    if (stop_tilt == true && (target_tilt != previous_target_tilt)) {
      stop_tilt = false;
      tilt_goal_reached = false;
    }
  }
}

void run_lift(double u) {
  lift_moving = true; //shows that the lift motor is moving
  if (u > 3)u = 3;
  if (u < -3)u = -3;
  double height_pwm = map_values(u, -3.0, 3.0, 1300, 1700); //1000us = clockwise, 2000us = counter-clockwise, map [-1000, 1000] to [1000, 2000]
  int height_pwm_int = (int)height_pwm;
  /*
    Serial.print("pwm: ");
    Serial.print(height_pwm);
    Serial.print(" u:" );
    Serial.println(u);
  */
  lift_motor.writeMicroseconds(height_pwm_int);
}

void run_tilt(double u) {
  tilt_moving = true; //shows that the lift motor is moving

  if (u > 100) { //cap value of u
    u = 100;
  } else if (u < -100) {
    u = -100;
  }

  double tilt_pwm = map_values(u, -100, 100, -255, 255); //1000us = clockwise, 2000us = counter-clockwise, map [-1000, 1000] to [1000, 2000]
  if (tilt_pwm <= lowest_tilt_pwm && tilt_pwm > 0) { //cap lowest speed
    tilt_pwm = lowest_tilt_pwm;
  } else if (tilt_pwm >= -lowest_tilt_pwm && tilt_pwm < 0) {
    tilt_pwm = -lowest_tilt_pwm;
  }

  // Set the speed and direction.
  if (tilt_pwm <= 0) { //go up
    analogWrite(kTiltPwmPin, -tilt_pwm);
    digitalWrite(kTiltDirectionPin, HIGH);
  } else { //go down
    analogWrite(kTiltPwmPin, tilt_pwm);
    digitalWrite(kTiltDirectionPin, LOW);
  }
}

void stop_motor_height() {
  lift_motor.writeMicroseconds(1500);
  lift_moving = false; //shows that the lift motor is not moving
}

void stop_motor_tilt() {
  tilt_moving = false; //shows that the lift motor is not moving
  analogWrite(kTiltPwmPin, 0);
  digitalWrite(kTiltDirectionPin, HIGH);
}

uint16_t getPositionSSI_efficient(uint8_t resolution) {
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

void stop_leg() {
  leg_moving = false;
  digitalWrite(kLegDirectionPin, LOW);
  analogWrite(kLegPwmPin, 0);
}

void run_leg(double leg_pid) {
  leg_moving = false;
  
  if (leg_pid > 2) {
    leg_pid = 2;
  }
  if (leg_pid < -2) {
    leg_pid = -2;
  }

  if (fabs(leg_pid) < (1.0 / 117.0)) {
    digitalWrite(kLegDirectionPin, LOW);
    analogWrite(kLegPwmPin, 0);
    leg_goal_reached = true;
    stop_leg();
    return;
  } else {
    double kLegPwmPin_double =  map_values(fabs(leg_pid), 0, 2.0, 0, 255.0);
    int kLegPwmPin_int = (int)kLegPwmPin_double;

    if (kLegPwmPin_int < 70 && kLegPwmPin_int > 0) {
      kLegPwmPin_int = 70;
    }
    if (kLegPwmPin_int > -70 && kLegPwmPin_int < 0) {
      kLegPwmPin_int = -70;
    }

    if (leg_pid < 0) {
      digitalWrite(kLegDirectionPin, HIGH);
      analogWrite(kLegPwmPin, kLegPwmPin_int);
      analogWrite(5, 255);
    }
    else {
      digitalWrite(kLegDirectionPin, LOW);
      analogWrite(kLegPwmPin, kLegPwmPin_int);
      analogWrite(5, 255);
    }
  }
}


double map_values(double value, double input_low, double input_high, double output_low, double output_high) {
  double f = (value - input_low) * (output_high - output_low) / (input_high - input_low) + output_low;
  return f;
}

void send_message(double height, double tilt, LegState leg) {
  Serial.print("T\t");
  Serial.print(tilt);
  Serial.print("\t");
  Serial.print(height);
  Serial.print("\t");
  Serial.print(leg);
  Serial.print("\tR");
  Serial.print("\n");
}

int poll_message(double &target_height, double &target_tilt, double &target_leg) {
  int i = 0;

  static char buf[200];
  memset(buf, 0, 200);

  long start_time = millis();

  while (Serial.available() > 0 && i < 200) {
    buf[i++] = Serial.read();
    if (millis() - start_time > 100) {
      return -3;
    }
  }

  if (i > 190) {
    while (Serial.available() > 0) {
      Serial.read();
    }
  }

  if (buf[0] == 'H' && buf[1] == 'i') {
    return 1;
  }

  if (buf[0] != 'T' && buf[1] != '\t') {
    Serial.println("BAD");

    return -1;
  }

  String height_str = "", tilt_str = "", leg_str = " ";

  int cnt = 2;

  while (buf[cnt] != '\t' && cnt < 199) {
    height_str += buf[cnt++];
  }
  cnt++;

  while (buf[cnt] != '\t' && cnt < 199) {
    tilt_str += buf[cnt++];
  }
  cnt++;

  while (buf[cnt] != '\t' && cnt < 199) {
    leg_str += buf[cnt++];
    leg_goal_reached = false;
  }
  cnt++;

  if (buf[cnt] != 'R') {
    Serial.println("BAD");
    return -2;
  }

  int N = 10;
  char height_buf[N], tilt_buf[N], leg_buf[N];

  height_str.toCharArray(height_buf, N);
  target_height = atof(height_buf);
  tilt_str.toCharArray(tilt_buf, N);
  target_tilt = atof(tilt_buf);
  leg_str.toCharArray(leg_buf, N);
  target_leg = atof(leg_buf);

  if (target_leg < 0.35) {
    target_leg = 0.35;
  }

  return 0;
}

/**
 * PID Controller Implementation
 */
 
PIDController::PIDController(double kP, double kI, double kD) {
  this->kP = kP;
  this->kI = kI;
  this->kD = kD;
  this->tol = 0.1;
}

double PIDController::set_tolerance(double tol) {
  this->tol = tol;
}

double PIDController::update(double current, double target) {
  double now = millis();
  double dt = prev_millis - now;
  prev_millis = now;
  double e = target - current;
  if (fabs(e) < tol) {
    at_target = true;
    has_target = false;
    return 0;
  }
  double d_e = (e - prev_error) / dt;
  integral_error += e * dt;
  double output = kP * e + kI * integral_error + kD * d_e;
  prev_error = e;
  return output;
}

bool PIDController::done() {
  return at_target;
}
