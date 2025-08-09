void setup() {
  // put your setup code here, to run once:
  pinMode(3, OUTPUT);
  pinMode(31, OUTPUT);
}

void moter_direction(int front){
  digitalWrite(31, front);
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
  moter_direction(1);
  moter_pwm();
}
//あほ //
// ahooo
