double target_height, target_tilt, target_leg;

char buf[200];

void setup() {
  Serial.begin(115200);
}

void loop() {
  
  send_message(target_height, target_tilt, target_leg);
  
  int rv = poll_message(target_height, target_tilt, target_leg);
  
  Serial.print(buf[2]);
  Serial.println(buf[3]);
  
  // check if this is a special "handshake" message from the python side
  if (buf[2] == 'H' && buf[3] == 'i'){
    Serial.println("Yo");
    return;
  }

  delay(10);
}

void send_message(double height, double tilt, double leg){
  return;
  Serial.print("T\t");
  Serial.print(tilt);
  Serial.print("\t");
  Serial.print(height);
  Serial.print("\t");
  Serial.print(leg);
  Serial.println("\tR");
  
}

int poll_message(double &target_height, double &target_tilt, double &target_leg){
  int i=0;
  memset(buf, 0, 200);
  
  while (Serial.available() > 0){
    buf[i++] = Serial.read();
  }

  /*
  
  String height_str = Serial.readStringUntil('\t');
  String tilt_str = Serial.readStringUntil('\t');
  String leg_str = Serial.readStringUntil('\t');
  String footer_str = Serial.readStringUntil('\t');
 
  if (footer_str != "R"){
     Serial.print("bad T: ");
     Serial.print(leg_str);
     Serial.print("\t");
     Serial.println(footer_str);
    return 1;
  }


  Serial.print(header_str);
  Serial.print(" ");
  Serial.print(height_str);
  Serial.print(" ");
  Serial.print(tilt_str);
  Serial.print(" ");
  Serial.print(leg_str);
  Serial.print(" ");
  Serial.println(footer_str);
  
  //flush_serial();  
  
  height_str.toCharArray(height_buf, N);
  target_height = atof(height_buf);
  tilt_str.toCharArray(tilt_buf, N);
  target_tilt = (atof(tilt_buf));
  leg_str.toCharArray(leg_buf, N);
  target_leg = atof(leg_buf);
  */
  
  return 0;
}

void flush_serial(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}   
