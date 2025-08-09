const int pwm_a = 2;
const int direction_a = 30;
const int pwm_b = 5;
const int direction_c = 33;
const int pwm_c = 3;
const int direction_b = 31;
const int pwm_d = 6;
const int direction_d = 34;




void setup() {
  // put your setup code here, to run once:
  pinMode(pwm_a, OUTPUT);
  pinMode(direction_a, OUTPUT);
  pinMode(pwm_b, OUTPUT);
  pinMode(direction_b, OUTPUT);
  pinMode(pwm_c, OUTPUT);
  pinMode(direction_c, OUTPUT);
  pinMode(pwm_d, OUTPUT);
  pinMode(direction_d, OUTPUT);
}

void moter_direction_A(int front){
  digitalWrite(direction_a, front);
}

void moter_direction_B(int front){
  digitalWrite(direction_b, front);
}

void moter_direction_C(int front){
  digitalWrite(direction_c, front);
}

void moter_direction_D(int front){
  digitalWrite(direction_d, front);
}

void loop() {
  // put your main code here, to run repeatedly:

}


void moter_AD(int on_off,int flont)
{
  if(on_off == 1) //動かすモーターを固定//
  {
    analogWrite(pwm_a,255);
    analogWrite(pwm_b,0);
    analogWrite(pwm_c,0);
    analogWrite(pwm_d,255);
    if(front == 1)//モーターの回転の向き//
    {
      moter_direction_A(HIGH);
      moter_direction_D(HIGH);
    }
    else
    {
      moter_direction_A(LOW);
      moter_direction_D(LOW);
    }
  } 
}


void moter_BC(int on_off,int front)
{
  if(on_off == 1)
  {
    analogWrite(pwm_a,0);
    analogWrite(pwm_b,255);
    analogWrite(pwm_c,255);
    analogWrite(pwm_d,0);
    if(front == 1)
    {
      moter_direction_B(HIGH);
      moter_direction_D(HIGH);
    }
    else
    {
      moter_direction_B(LOW);
      moter_direction_D(LOW);
    } 
  }
}