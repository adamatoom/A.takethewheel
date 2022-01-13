
int pwm_value;
int val = 0;
void setup() {
  
  // put your setup code here, to run once:
//  analogWriteResolution(12);
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  pinMode(13, OUTPUT);
  pinMode(2, INPUT);
}

  int vol = 0;
void loop() {
if (Serial1.available()) {
  vol = Serial1.read()-127;
//Serial.println(vol);  
}
if (Serial2.available()) {
  vol = Serial2.read()-127;
Serial.println(vol);  
}
  analogWrite(DAC0, 191 - vol / 2.0 );
//  analogWrite(DAC0, 255 );
  analogWrite(DAC1, 191 + vol / 2.0 );
//Serial.println( vol / 5.0 * 2.0 * 3.3 / 255.0);  
val = analogRead(A0);  // read the input pin
//  Serial.println(val);  
  // put your main code here, to run repeatedly:
  int i = 1;
  bool f = 1;

}
