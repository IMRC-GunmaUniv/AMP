#include <Servo.h>
Servo servo1;
const int servo1Pin = 12;  // servo1 接 Pin 11,12,13
int servo1Dir = 1;
int posOfServo1 = 0;
int servo_millis_timer = millis();
int servo_interval = 50;

int servo_max = 270;
int servo_min = 0;

const int MabuchimoterPWM = 4;   // モータのPWMピン
const int MabuchimotorDir = 32;  // モータの方向制御）
int MabuchimotorSpeed = 30;      // PWM出力


const int limit_pin[2] = { 38, 36 };
int Mab_dir = HIGH;
int pre_limit_state[2] = { 0, 0 };
int MabuchiNeutral = 0;
int MabuchiStop[2] = { 0, 0 };


//それぞれのボタンの定義
const String wiredControllerMap[] = {
  "UP", "LEFT", "DOWN", "RIGHT",
  "LUP", "LLEFT", "LDOWN", "LRIGHT", "LS",
  "L1", "L2",
  "UNBIND", "UNBIND", "UNBIND", "UNBIND", "UNBIND",
  "X", "Y", "B", "A",
  "RUP", "RLEFT", "RDOWN", "RRIGHT", "RS",
  "R1", "R2",
  "UNBIND", "UNBIND", "UNBIND", "UNBIND", "UNBIND"
};

//                u  d  l  r  a  b  x  y l1 r1 l2 r2 ls rs
int btnState[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

// xは右にやると+、yは下にやると+
// -5 ~ 5(ESP32側のプログラムで変更可能)
//               lx ly rx ry
int axiState[] = { 0, 0, 0, 0 };




void setup() {
  servo1.attach(servo1Pin, 500, 2500);
  servo1.write(posOfServo1);

  pinMode(limit_pin[0], INPUT);
  pinMode(limit_pin[1], INPUT);
  pinMode(MabuchimoterPWM, OUTPUT);
  pinMode(MabuchimotorDir, OUTPUT);





  //無線通信
  Serial1.begin(115200);    // ESP用
  SerialUSB.begin(115200);  // PC
}








//無線
//指定したボタンが入力されているかの確認
int getBtnState(String key) {
  // getBtnState("A")で、〇ボタンのon/offが返ってくる
  // 押されてれば1、押されてなければ0


  String keyMap[] = { "UP", "DOWN", "LEFT", "RIGHT", "A", "B", "X", "Y", "L1", "R1", "L2", "R2" };

  for (int i = 0; i < 12; i++) {
    if (keyMap[i] == key) {
      return btnState[i];
    }
  }

  return 0;
}



//無線通信するために必要なもの
void parseCtlState() {
  // ESP32からの通信を解析し、コントローラーの配列を設定

  // 改行コードが来るまでバッファに入れ、改行コードを削除
  String data = Serial1.readStringUntil(0x0a);
  data.trim();

  // "DATA: "から始まらないものは解析しない
  // "INFO: Controller has disconnected"とかそういうメッセージなので、そのままPC側に送る
  if (!data.startsWith("DATA: ")) {
    Serial.println(data);
    return;
  }

  data.replace("DATA: ", "");

  int len = data.length();

  // data = "0 0 1 1 0 0 0 0 1 0 0 0 3 -2 0 4"
  // コントローラの状況の文字列データを空白で切り分け、数字として仮の配列に格納
  String buf = "";
  int idx = 0;
  int ctlState[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  for (int i = 0; i < len; i++) {
    char cursor = data.charAt(i);

    if (cursor == ' ') {
      ctlState[idx] = buf.toInt();

      buf = "";
      idx++;
      continue;
    }

    buf.concat(cursor);
  }

  ctlState[17] = buf.toInt();

  // 仮の配列からほかでも使うグローバルの配列に格納
  for (int i = 0; i < 14; i++) {
    btnState[i] = ctlState[i];
  }
  for (int i = 0; i < 4; i++) {
    axiState[i] = ctlState[i + 14];
  }
}








// --- サーボ制御 ---
void servo_controle() {
  if (getBtnState("R1") == 1 || getBtnState("R1") == 1) {
    if (posOfServo1 > servo_min) {
      posOfServo1 -= servo1Dir;
    }
  }
  if (getBtnState("L1") == 1 || getBtnState("L1") == 1) {
    if (posOfServo1 < servo_max) {
      posOfServo1 += servo1Dir;
    }
  }

  if (getBtnState("L1") == 0 || getBtnState("R1") == 0) {
  }

  servo1.write(posOfServo1);
}

// ---- モーター制御 ----
void motor_control() {
  if (getBtnState("L2") == 0 && getBtnState("R2") == 0) {
    //リミットスイッチが押された瞬間、操作を受け付けなくする。L2もR2も押されていない状態でリセット
    MabuchiNeutral = 0;
    analogWrite(MabuchimoterPWM, 0);
  }
  if (MabuchiNeutral == 0) {
    if (getBtnState("R2") == 1 && MabuchiStop[1] == 0) {
      Mab_dir = LOW;
      analogWrite(MabuchimoterPWM, MabuchimotorSpeed);
    }
    if (getBtnState("L2") == 1 && MabuchiStop[0] == 0) {
      Mab_dir = HIGH;
      analogWrite(MabuchimoterPWM, MabuchimotorSpeed);
    }
  } else {
    analogWrite(MabuchimoterPWM, 0);
  }
  digitalWrite(MabuchimotorDir, Mab_dir);
}

// ---- リミットスイッチ ----
void limit_check() {
  for (int i = 0; i < 2; i++) {
    int limit_state = digitalRead(limit_pin[i]);
    if (limit_state == HIGH) {
      if (limit_state != pre_limit_state[i]) {
        //リミットスイッチが押されたその瞬間のみこのifに入る
        MabuchiNeutral = 1; //モーターを完全に動かなくする
      }
      //要求以上に前に動かなくする
      MabuchiStop[i] = 1;
    } else {
      MabuchiStop[i] = 0;
    }
    pre_limit_state[i] = limit_state;
  }
}

void loop() {
  if (Serial1.available()) {
    parseCtlState();
  }

  if (millis() - servo_millis_timer >= servo_interval) {
    servo_controle();
    servo_millis_timer = millis();
  }

  motor_control();
  limit_check();
}