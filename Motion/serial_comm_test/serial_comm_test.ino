double target_height, target_tilt, target_leg;

char buf[200];

void setup() {
  Serial.begin(115200);
}

void loop() {
    
  int rv = poll_message(target_height, target_tilt, target_leg);
  
  // check if this is a special "handshake" message from the python side
  if (rv == 1){
    Serial.print("yo\n");
  } else if (rv == 0){
    send_message(target_height, target_tilt, target_leg);
  } else {
    Serial.print("BAD\n");
  }

  delay(10);
}

void send_message(double height, double tilt, double leg){
  Serial.print("T\t");
  Serial.print(tilt);
  Serial.print("\t");
  Serial.print(height);
  Serial.print("\t");
  Serial.print(leg);
  Serial.print("\tR");
  Serial.print("\n");
  
}

int poll_message(double &target_height, double &target_tilt, double &target_leg){
  int i=0;
  memset(buf, 0, 200);
  
  while (Serial.available() > 0 && i < 200){
    buf[i++] = Serial.read();
  }

  if (i > 190){
    while (Serial.available() > 0){
      Serial.read();
    }
  }

  if (buf[0] == 'H' && buf[1] == 'i'){
    return 1;
  }
  
  if (buf[0] != 'T' && buf[1] != '\t'){
    return -1;
  }

  String height_str = "", tilt_str = "", leg_str =" ";

  int cnt = 2;
  
  while(buf[cnt] != '\t' && cnt < 199){
    height_str += buf[cnt++]; 
  }
  cnt++;

  while(buf[cnt] != '\t' && cnt < 199){
    tilt_str += buf[cnt++]; 
  }
  cnt++;

  while(buf[cnt] != '\t' && cnt < 199){
    leg_str += buf[cnt++]; 
  }
  cnt++;

  if (buf[cnt] != 'R'){
    return -2;
  }

  int N = 10;
  char height_buf[N], tilt_buf[N], leg_buf[N];

  height_str.toCharArray(height_buf, N);
  target_height = atof(height_buf);
  tilt_str.toCharArray(tilt_buf, N);
  target_tilt = (atof(tilt_buf));
  leg_str.toCharArray(leg_buf, N);
  target_leg = atof(leg_buf);
  
  return 0;
}

void flush_serial(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}   
