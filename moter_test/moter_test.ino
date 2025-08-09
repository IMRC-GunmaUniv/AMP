void setup() {
  // put your setup code here, to run once:
  pinMode(3, OUTPUT);
  pinMode(31, OUTPUT);
}

void moter_direction(){
  digitalWrite(31, HIGH);
}

void moter_pwm(){
  analogWrite(3, 256);
  delay(1000);
  analogWrite(3, 128);
  delay(1000);
  analogWrite(3, 0);
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  moter_direction();
  moter_pwm();
}
