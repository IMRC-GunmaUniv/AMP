//モーターの回転数の平均を出すためのコード。〇ボタンを押したら計測開始

//ピン
//abcdのpwmとdirectonの数の決定
const int pwm_a = 3;
const int direction_a = 31;
const int pwm_b = 2;
const int direction_b = 30;
const int pwm_c = 6;
const int direction_c = 34;
const int pwm_d = 5;
const int direction_d = 33;
//エンコーダ
const int encoder_a1 = 41;
const int encoder_a2 = 42;
const int encoder_b1 = 46;
const int encoder_b2 = 47;
const int encoder_c1 = 39;
const int encoder_c2 = 40;
const int encoder_d1 = 44;
const int encoder_d2 = 45;

int old_position_a = digitalRead(encoder_a1);
int old_position_b = digitalRead(encoder_b1);
int old_position_c = digitalRead(encoder_c1);
int old_position_d = digitalRead(encoder_d1);

int moter_speed = 200;

int moter_move_check = 0;

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

int moter_enc_list[4] = { 0, 0, 0, 0 };

void setup() {
  // put your setup code here, to run once:
  //pinModeでそれぞれのモーターを定義//
  pinMode(pwm_a, OUTPUT);
  pinMode(direction_a, OUTPUT);
  pinMode(pwm_b, OUTPUT);
  pinMode(direction_b, OUTPUT);
  pinMode(pwm_c, OUTPUT);
  pinMode(direction_c, OUTPUT);
  pinMode(pwm_d, OUTPUT);
  pinMode(direction_d, OUTPUT);

  //無線通信
  Serial.begin(115200);  // PC
  Serial1.begin(115200);  // ESP

  //PID
  //moter_a
  pinMode(encoder_a1, INPUT);
  pinMode(encoder_a2, INPUT);
  //割り込み関数の定義
  attachInterrupt(digitalPinToInterrupt(encoder_a1), encoder_a, CHANGE);
  //moter_b
  pinMode(encoder_b1, INPUT);
  pinMode(encoder_b2, INPUT);
  //割り込み関数の定義
  attachInterrupt(digitalPinToInterrupt(encoder_b1), encoder_b, CHANGE);
  //moter_c
  pinMode(encoder_c1, INPUT);
  pinMode(encoder_c2, INPUT);
  //割り込み関数の定義
  attachInterrupt(digitalPinToInterrupt(encoder_c1), encoder_c, CHANGE);
  //moter_d
  pinMode(encoder_d1, INPUT);
  pinMode(encoder_d2, INPUT);
  //割り込み関数の定義
  attachInterrupt(digitalPinToInterrupt(encoder_d1), encoder_d, CHANGE);

  Serial.println();
  Serial.println("start");
}


void encoder_a() {
  int new_position_a = digitalRead(encoder_a1);
  if (new_position_a != old_position_a) {
    //↓のifを消せばよさそう
    if (digitalRead(encoder_a2) != new_position_a) {
      moter_enc_list[0]--;
    } else {
      moter_enc_list[0]++;
    }
    moter_move_check = 1;
    old_position_a = new_position_a;
  }
}

void encoder_b() {
  int new_position_b = digitalRead(encoder_b1);
  if (new_position_b != old_position_b) {
    if (digitalRead(encoder_b2) != new_position_b) {
      moter_enc_list[1]--;
    } else {
      moter_enc_list[1]++;
    }
    moter_move_check = 1;
    old_position_b = new_position_b;
  }
}

void encoder_c() {
  int new_position_c = digitalRead(encoder_c1);
  if (new_position_c != old_position_c) {
    if (digitalRead(encoder_c2) != new_position_c) {
      moter_enc_list[2]++;
    } else {
      moter_enc_list[2]--;
    }
    moter_move_check = 1;
    old_position_c = new_position_c;
  }
}

void encoder_d() {
  int new_position_d = digitalRead(encoder_d1);
  if (new_position_d != old_position_d) {
    if (digitalRead(encoder_d2) != new_position_d) {
      moter_enc_list[3]++;
    } else {
      moter_enc_list[3]--;
    }
    moter_move_check = 1;
    old_position_d = new_position_d;
  }
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


//指定した方向にスティックが倒されているかの確認
int getAxiState(String key, bool isBin = false) {
  // getAxiState("LY")で、スティックの軸の正規化された値が返ってくる
  // Xは右が+、Yは下が+
  // 範囲はESPのプログラムによって変わる。-4 ~ 4？y軸については上に上がるほど低い値をとる（－4に近づくという点に注意）


  String keyMap[] = { "LX", "LY", "RX", "RY" };
  int binThreshold = 2;

  for (int i = 0; i < 4; i++) {
    if (keyMap[i] == key) {
      if (isBin) {
        if (axiState[i] >= binThreshold) {
          return 1;
        } else if (axiState[i] <= -binThreshold) {
          return -1;
        } else {
          return 0;
        }
      } else {
        return axiState[i];
      }
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

int one_second = 1000000;
int count = 1;
int check = 0;
int timer = 1000000;
int avr = 0;

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial1.available()) {
    //接続確認用（このif文は消さないで）
    parseCtlState();
  }

  digitalWrite(direction_a, HIGH);
  digitalWrite(direction_b, HIGH);
  digitalWrite(direction_c, HIGH);
  digitalWrite(direction_d, HIGH);

  if (getBtnState("A") == 1 && check == 0) {
    check = 1;
    timer = millis();
    one_second = millis();
  }

  if (check == 1) {
    analogWrite(pwm_a, moter_speed);
    analogWrite(pwm_b, moter_speed);
    analogWrite(pwm_c, moter_speed);
    analogWrite(pwm_d, moter_speed);
    if (millis() - timer > 10000) {
      Serial.println();
      Serial.println("finish");
      Serial.print("A:");
      Serial.print(moter_enc_list[0]);
      Serial.print(" B:");
      Serial.print(moter_enc_list[1]);
      Serial.print(" C:");
      Serial.print(moter_enc_list[2]);
      Serial.print(" D:");
      Serial.println(moter_enc_list[3]);
      for (int i = 0; i < 4; i++) {
        avr += abs(moter_enc_list[i]);
      }
      avr /= 4;
      Serial.print("ten second:");
      Serial.println(avr);
      Serial.print("one second:");
      Serial.println(avr / 10);
      check = 0;

    } else if (millis() - one_second > 1000) {
      Serial.println(count);
      Serial.print("A:");
      Serial.print(moter_enc_list[0]);
      Serial.print(" B:");
      Serial.print(moter_enc_list[1]);
      Serial.print(" C:");
      Serial.print(moter_enc_list[2]);
      Serial.print(" D:");
      Serial.println(moter_enc_list[3]);
      count += 1;
      one_second = millis();
    }
  } else {
    analogWrite(pwm_a, 0);
    analogWrite(pwm_b, 0);
    analogWrite(pwm_c, 0);
    analogWrite(pwm_d, 0);
  }
}

/*
一秒間での回転量
計測一回目：1197
計測二回目：1196
計測三回目：1195
計測四回目：1194
計測五回目：1193
*/