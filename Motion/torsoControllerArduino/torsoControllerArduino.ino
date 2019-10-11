void send_message(double height, double tilt, bool lift_limit_switch, bool tilt_limit_switch){
  // serialized data format: "TRINA\t[tilt]\t[height]\t[tilt_limit_switch]\t[lift_limit_switch]\tTRINA\n"
  Serial.print("TRINA\t");
  Serial.print(tilt);
  Serial.print("\t");
  Serial.print(height);
  Serial.print("\t");
  Serial.print(tilt_limit_switch);
  Serial.print("\t");
  Serial.print(lift_limit_switch);
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

void setup() {
  Serial.begin(9600);
}

void loop() {
  //double height = 12305.323432;
  //double tilt = 1234.56789;
  double height, tilt;
  poll_message(height, tilt);
  bool lift_limit_switch = true;
  bool tilt_limit_switch = false;
  send_message(height, tilt, lift_limit_switch, tilt_limit_switch);
  delay(1.0);
}
