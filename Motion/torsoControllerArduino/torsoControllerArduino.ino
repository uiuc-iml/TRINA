void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
}

void loop() {
  float height = 12305.323432;
  float tilt = 1234.56789;
  // serialized data format: "TRINA\t[tilt]\t[height]\tTRINA\n"
  Serial.print("TRINA\t");
  Serial.print(tilt);
  Serial.print("\t");
  Serial.print(height);
  Serial.print("\t");
  Serial.println("TRINA");
  delay(1.0);
}
