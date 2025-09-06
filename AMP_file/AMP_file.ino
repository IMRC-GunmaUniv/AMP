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
const int encoder_c1 = 44;
const int encoder_c2 = 45;
const int encoder_d1 = 39;
const int encoder_d2 = 40;
//タクトスイッチ
const int tact[4] = { 28, 26, 29, 27 };
//LED
const int LED[4] = { 25, 24, 23, 22 };
//射出
const int valve_a = 50;
const int valve_b = 51;


//モーターの速度の目標値（スティックが完全に倒されている時）
int max_straight_speed = 200;
int max_diagonal_speed = 200;
int max_spin_speed = 200;

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

//               { u  d  l  r  a  b  x  y l1 r1 l2 r2 ls rs }
int btnState[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

// xは右にやると+、yは下にやると+
// -5 ~ 5(ESP32側のプログラムで変更可能)
//               lx ly rx ry
int axiState[] = { 0, 0, 0, 0 };

//左スティックの状態を表す変数
int lx_state = 0;
int ly_state = 0;
int rx_state = 0;


//PID
//基準処理（主モーター）
const float Kp_moter_base = 0.5;
const float Ki_moter_base = 0.5;
const float Kd_moter_base = 0.1;
//analogWrite1あたりのモーターの回転数
const int Rotate_Unit = 6;
//一秒間で回転するモーターの目標値。analogWriteの引数1あたりの一秒間の回転数は6。digitalWriteの引数が200の時は1200とする。
//この値はmeasurement_fileを用いて出した値
int Target_RPM_moter = 0;
//前回のエンコーダーの値
//a
int pre_encoder_a = 0;
//b
int pre_encoder_b = 0;
//PID制御用のタイマー。現在は一秒間に一回行う設定。初期化の値は-1000が好ましい。（動かしてすぐにif文の中に入れるように）
int pid_timer_base = -1000;
//
int moter_base_speed = 0;
//積分で使う用の誤差の合計
int base_error_list = 0;


//同期処理（他のホイール）
int moter_move_check = 0;
int master_encoder = 0;
const float Kp_moter_sync = 1;
const float Ki_moter_sync = 0.5;
const float Kd_moter_sync = 0.1;
int pid_timer_sync = -1000;
int old_position_a = digitalRead(encoder_a1);
int old_position_b = digitalRead(encoder_b1);
int old_position_c = digitalRead(encoder_c1);
int old_position_d = digitalRead(encoder_d1);
//それぞれのモーターのエンコーダー。左から a  b  c  d
int moter_enc_list[4] = { 0, 0, 0, 0 };
//それぞれのモーターに渡す出力の値。左から a  b  c  d
int moter_power_list[4] = { 0, 0, 0, 0 };
//それぞれのモーターの誤差の合計。左から a  b  c  d
int moter_error_total[4] = { 0, 0, 0, 0 };

int sync_error_list[4] = { 0, 0, 0, 0 };
int pre_sync_error_list[4] = { 0, 0, 0, 0 };
const float MAX_INTEGRAL = 20;                  //I制御における値を調整（範囲）
const float DeadLine = 1;                       //I制御における値を調整（誤差の消去）
const float alpha = 0.8;                        // 0.0〜1.0で調整（小さいほど滑らか）
float smoothed_derivative[4] = { 0, 0, 0, 0 };  //D制御のフィルター

//確認用
int tact_checker[4] = { LOW, LOW, LOW, LOW };
int pre_tact_state[4] = { LOW, LOW, LOW, LOW };
int debug_timer = 0;

void setup() {
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
  Serial1.begin(115200);    // ESP用
  SerialUSB.begin(115200);  // PC

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

  //射出
  pinMode(valve_a, OUTPUT);
  pinMode(valve_b, OUTPUT);

  //確認用
  //タクトスイッチ
  for (int i = 0; i < 4; i++) {
    //タクトスイッチ
    pinMode(tact[i], INPUT);
    //LED

    
    pinMode(LED[i], OUTPUT);
  }

  SerialUSB.println();
  SerialUSB.println("start");
}

//モーター
//それぞれのモーターの回転の向きを定義//
void moter_direction_A(int front) {
  digitalWrite(direction_a, front);
}

void moter_direction_B(int front) {
  digitalWrite(direction_b, front);
}

void moter_direction_C(int front) {
  digitalWrite(direction_c, front);
}

void moter_direction_D(int front) {
  digitalWrite(direction_d, front);
}



void moter_front(int on_off, int front, int master_moter_power) {
  //前か後ろに移動
  if (on_off == 1) {
    moter_pid_sync("a", master_moter_power);
    analogWrite(pwm_a, moter_power_list[0]);
    analogWrite(pwm_b, moter_power_list[1]);
    analogWrite(pwm_c, moter_power_list[2]);
    analogWrite(pwm_d, moter_power_list[3]);
    if (front == 1) {
      moter_direction_A(LOW);
      moter_direction_B(HIGH);
      moter_direction_C(LOW);
      moter_direction_D(HIGH);
    } else {
      moter_direction_A(HIGH);
      moter_direction_B(LOW);
      moter_direction_C(HIGH);
      moter_direction_D(LOW);
    }
  }
}


void moter_right(int on_off, int front, int master_moter_power) {
  //右か左に移動

  if (on_off == 1) {
    moter_pid_sync("a", master_moter_power);
    analogWrite(pwm_a, moter_power_list[0]);
    analogWrite(pwm_b, moter_power_list[1]);
    analogWrite(pwm_c, moter_power_list[2]);
    analogWrite(pwm_d, moter_power_list[3]);
    if (front == 1) {
      moter_direction_A(LOW);
      moter_direction_B(LOW);
      moter_direction_C(HIGH);
      moter_direction_D(HIGH);
    } else {
      moter_direction_A(HIGH);
      moter_direction_B(HIGH);
      moter_direction_C(LOW);
      moter_direction_D(LOW);
    }
  }
}


void moter_BC(int on_off, int front, int master_moter_power)  //斜めに動く関数//
{
  if (on_off == 1)  //動かすモーターを固定//
  {
    moter_pid_sync("b", master_moter_power);
    analogWrite(pwm_a, 0);
    analogWrite(pwm_b, moter_power_list[1]);
    analogWrite(pwm_c, moter_power_list[2]);
    analogWrite(pwm_d, 0);
    if (front == 1)  //モーターの回転の向き//
    {
      moter_direction_B(HIGH);
      moter_direction_C(LOW);
    } else {
      moter_direction_B(LOW);
      moter_direction_C(HIGH);
    }
  }
}


void moter_AD(int on_off, int front, int master_moter_power)  //斜めに動く関数//
{
  if (on_off == 1) {
    moter_pid_sync("a", master_moter_power);
    analogWrite(pwm_a, moter_power_list[0]);
    analogWrite(pwm_b, 0);
    analogWrite(pwm_c, 0);
    analogWrite(pwm_d, moter_power_list[3]);
    if (front == 1) {
      moter_direction_A(LOW);
      moter_direction_D(HIGH);
    } else {
      moter_direction_A(HIGH);
      moter_direction_D(LOW);
    }
  }
}


void moter_spin(int on_off, int left, int master_moter_power)  //回転する関数//
{
  if (on_off == 1) {
    moter_pid_sync("a", master_moter_power);
    analogWrite(pwm_a, moter_power_list[0]);
    analogWrite(pwm_b, moter_power_list[1]);
    analogWrite(pwm_c, moter_power_list[2]);
    analogWrite(pwm_d, moter_power_list[3]);
    if (left == 1) {
      moter_direction_A(HIGH);
      moter_direction_B(HIGH);
      moter_direction_C(HIGH);
      moter_direction_D(HIGH);
    } else {
      moter_direction_A(LOW);
      moter_direction_B(LOW);
      moter_direction_C(LOW);
      moter_direction_D(LOW);
    }
  }
}


void moter_initialization() {
  //モータの動きを初期化
  moter_front(0, 0, 0);
  moter_right(0, 0, 0);
  moter_BC(0, 0, 0);
  moter_AD(0, 0, 0);
  moter_spin(0, 0, 0);
  analogWrite(pwm_a, 0);
  analogWrite(pwm_b, 0);
  analogWrite(pwm_c, 0);
  analogWrite(pwm_d, 0);
  moter_pid_sync("nothing", 0);
  for (int i = 0; i < 4; i++) {
    moter_enc_list[i] = 0;
    moter_error_total[i] = 0;
  }
  pid_timer_sync = 0;
  pid_timer_base = millis();
  pre_encoder_a = moter_enc_list[0];
  pre_encoder_b = moter_enc_list[1];
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
    SerialUSB.println(data);
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



//コントローラーで左スティックが倒されたときに機体が動くようにする
void controller_move() {
  if ((lx_state == 0 && ly_state == 0 && rx_state == 0) || (lx_state != getAxiState("LX") || ly_state != getAxiState("LY"))) {
    moter_initialization();
  }
  lx_state = getAxiState("LX");
  ly_state = getAxiState("LY");

  //前後左右
  if (lx_state == 0) {
    //縦移動
    if (ly_state < 0) {
      //下
      moter_front(HIGH, HIGH, wheel_speed_left());
    }
    if (ly_state > 0) {
      //上
      moter_front(HIGH, LOW, wheel_speed_left());
    }
  }
  if (ly_state == 0) {
    //横移動
    if (lx_state > 0) {
      //右
      moter_right(HIGH, HIGH, wheel_speed_left());
    }
    if (lx_state < 0) {
      //左
      moter_right(HIGH, LOW, wheel_speed_left());
    }
  }
  //斜め
  if (lx_state != 0 && lx_state != 0) {
    if (lx_state > 0 && ly_state < 0) {
      //右斜め前
      moter_AD(HIGH, HIGH, wheel_speed_left());
    }
    if (lx_state < 0 && ly_state > 0) {
      //左斜め下
      moter_AD(HIGH, LOW, wheel_speed_left());
    }
    if (lx_state < 0 && ly_state < 0) {
      //左斜め前
      moter_BC(HIGH, HIGH, wheel_speed_left());
    }
    if (lx_state > 0 && ly_state > 0) {
      //右斜め下
      moter_BC(HIGH, LOW, wheel_speed_left());
    }
  }
}


//コントローラーで右スティックが倒されたときに機体が回転するようにする
void controller_spin() {
  if ((rx_state == 0 && lx_state == 0 && ly_state == 0) || rx_state != getAxiState("RX")) {
    moter_initialization();
  }
  rx_state = getAxiState("RX");
  if (rx_state > 0) {
    moter_spin(HIGH, HIGH, wheel_speed_right());
  }
  if (rx_state < 0) {
    moter_spin(HIGH, LOW, wheel_speed_right());
  }
}


//左スティックの倒す度合いによって機体の速度が増減する
int wheel_speed_left() {
  if (lx_state == 0) {
    int left_absolute_value = abs(ly_state);
    if (left_absolute_value == 4) {
      return max_straight_speed;
    }
    if (left_absolute_value == 3) {
      return max_straight_speed / 2;
    }
    if (left_absolute_value == 2) {
      return max_straight_speed / 3;
    }
    if (left_absolute_value == 1) {
      return max_straight_speed / 4;
    }
  }
  if (ly_state == 0) {
    int left_absolute_value = abs(lx_state);
    if (left_absolute_value == 4) {
      return max_straight_speed;
    }
    if (left_absolute_value == 3) {
      return max_straight_speed / 2;
    }
    if (left_absolute_value == 2) {
      return max_straight_speed / 3;
    }
    if (left_absolute_value == 1) {
      return max_straight_speed / 4;
    }
  }
  if (lx_state != 0 && ly_state != 0) {
    int xy_coordinate = lx_state * lx_state + ly_state * ly_state;
    if (xy_coordinate <= 25 && xy_coordinate > 13) {
      return max_diagonal_speed;
    }
    if (xy_coordinate <= 13 && xy_coordinate > 5) {
      return max_diagonal_speed / 2;
    }
    if (xy_coordinate <= 5) {
      return max_diagonal_speed / 3;
    }
  }
}


//右スティックの倒す度合いによって機体の回転速度が増減する
int wheel_speed_right() {
  if (abs(rx_state) == 4) {
    return max_spin_speed;
  }
  if (abs(rx_state) == 3) {
    return max_spin_speed / 2;
  }
  if (abs(rx_state) == 2) {
    return max_spin_speed / 3;
  }
  if (abs(rx_state) == 1) {
    return max_spin_speed / 4;
  }
}



//PID
//エンコーダー
//割り込み関数を用いてエンコーダーの値が変更されたとき回転数の総和を導く
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


//基準処理（主モーター）
int moter_pid_base(String master_moter_name, int master_speed) {
  if ((master_moter_name.equals("a") || master_moter_name.equals("b")) && tact_checker[1] == LOW) {
    //1秒に一回制御を行う
    if (millis() - pid_timer_base > 1000) {

      //タイマーの初期化
      pid_timer_base = millis();

      //スティックの倒され具合によって目標値を設定
      if (master_speed == 200) {
        Target_RPM_moter = 1200;
      } else if (master_speed == 100) {
        Target_RPM_moter = 600;
      } else if (master_speed == 66) {
        Target_RPM_moter = 400;
      } else if (master_speed == 50) {
        Target_RPM_moter = 300;
      }

      //SerialUSB.println(moter_proportional_base(master_moter_name));
      moter_base_speed = master_speed + Kp_moter_base * moter_proportional_base(master_moter_name) + Ki_moter_base * moter_integral_base(master_moter_name) + Kd_moter_base * moter_differential_base(master_moter_name);

      /*
      SerialUSB.print("  speed: ");
      SerialUSB.println(moter_base_speed);
*/

      //異常な値を与えないようにする
      if (moter_base_speed > 255) {
        return 255;
      } else if (moter_base_speed < 0) {
        return 0;
      }

      return moter_base_speed;

    } else {

      //制御を行ってから1秒に満たない場合、前回の数値を出力
      return moter_base_speed;
    }

  } else {
    //PID制御を用いない時用。基本は第一引数に"nothing"を代入すること
    return master_speed;
  }
}

//P制御
int moter_proportional_base(String master_moter_name) {
  int one_second_encoder;
  if (master_moter_name.equals("a")) {
    //1秒間あたりのモーターの回転数
    one_second_encoder = moter_enc_list[0] - pre_encoder_a;
    //現在の値を1秒後に使えるようにする
    pre_encoder_a = moter_enc_list[0];
  } else {
    one_second_encoder = moter_enc_list[1] - pre_encoder_b;
    pre_encoder_b = moter_enc_list[1];
  }
  //SerialUSB.print(one_second_encoder);
  //目標値の誤差÷6　6はanalogWriteに与える第二引数1あたりのモーターの回転量
  return (Target_RPM_moter - one_second_encoder) / Rotate_Unit;
}

//I制御
/*int moter_integral_base(String master_moter_name) {
    if(master_moter_name.equals("a")) {
       int one_power = moter_proportional_base() - pre_power_a;
       pre_power_a = moter_proportional_base();
       

    }
   

  }
}*/

//D制御



int moter_integral_base(String master_moter_name) {
  if (master_moter_name.equals("a")) {
    return 0;
  } else {
    return 0;
  }
}

//D制御
int moter_differential_base(String master_moter_name) {
  if (master_moter_name.equals("a")) {
    return 0;
  } else {
    return 0;
  }
}

//同期処理（他のモータ）
void moter_pid_sync(String master_moter_name, int master_speed) {
  if ((master_moter_name == "a" || master_moter_name == "b") && tact_checker[1] == LOW) {

    moter_proportional_sync(master_moter_name);

    if (millis() - pid_timer_sync > 1000) {


      moter_integral_sync();
      moter_differential_sync();

      pid_timer_sync = millis();
    }
    //↓でanalog.writeがとりうる値を超えないようにしている
    for (int i = 0; i < 4; i++) {
      moter_power_list[i] += master_speed;
      //SerialUSB.println(moter_power_list[i]);
      if (moter_power_list[i] > 255) {
        moter_power_list[i] = 255;
      } else if (moter_power_list[i] < 0) {
        moter_power_list[i] = 0;
      }
    }

  } else {
    //PID制御を用いない時用。基本は第一引数に"nothing"を代入すること
    for (int i = 0; i < 4; i++) {
      moter_power_list[i] = master_speed;
    }
  }
}


//P制御
void moter_proportional_sync(String master_moter_name) {
  //それぞれのホイールのスピードをP制御を用いたうえで出力。moter_power_listというリストにそれぞれのスピードを代入しています。
  if (master_moter_name == "a") {
    master_encoder = abs(moter_enc_list[0]);  //目標値の設定（この制御では一つのホイールの値を目標値とし、その他のホイールをその値に合わせる）
  } else if (master_moter_name == "b") {
    master_encoder = abs(moter_enc_list[1]);
  }
  for (int i = 0; i < 4; i++) {
    sync_error_list[i] = master_encoder - abs(moter_enc_list[i]);
    moter_power_list[i] = Kp_moter_sync * sync_error_list[i];
    //SerialUSB.println(moter_power_list[i]);
  }
}


//I制御
void moter_integral_sync() {
  for (int i = 0; i < 4; i++) {
    if (abs(sync_error_list[i]) < DeadLine) {
      //SerialUSB.println("continue");
      continue;  //誤差が小さいときは積分しない
    } else {
      moter_error_total[i] += sync_error_list[i];  // 誤差を積分
    }

    moter_error_total[i] = constrain(moter_error_total[i], -MAX_INTEGRAL, MAX_INTEGRAL);  //誤差の値を制限
    moter_power_list[i] += Ki_moter_sync * moter_error_total[i];
    //SerialUSB.println(Ki_moter_sync * moter_error_total[i]);
  }
}

//D制御
void moter_differential_sync() {
  for (int i = 0; i < 4; i++) {
    float derivative = sync_error_list[i] - pre_sync_error_list[i];
    pre_sync_error_list[i] = sync_error_list[i];
    smoothed_derivative[i] = alpha * smoothed_derivative[i] + (1 - alpha) * derivative;
    moter_power_list[i] += Kd_moter_sync * smoothed_derivative[i];
    //SerialUSB.println(smoothed_derivative[i]);
  }
}

//射出
void injection() {
  if (getBtnState("A") == 1 && getBtnState("Y") == 0) {
    //SerialUSB.println("50 HIGH");
    digitalWrite(50, LOW);
    digitalWrite(51, HIGH);
  }
  if (getBtnState("Y") == 1 && getBtnState("A") == 0) {
    //SerialUSB.println("51 HIGH");
    digitalWrite(50, HIGH);
    digitalWrite(51, LOW);
  }
  if (getBtnState("Y") == 0 && getBtnState("A") == 0) {
    //SerialUSB.println("LOW");
    digitalWrite(valve_a, HIGH);
    digitalWrite(valve_b, HIGH);
  }
}



//確認用
void confirmation() {
  tact_check();
  debug();
}

void tact_check() {
  for (int i = 0; i < 4; i++) {
    int tact_state = digitalRead(tact[i]);
    if (tact_state == LOW && tact_state != pre_tact_state[i]) {
      if (tact_checker[i] == LOW) {
        tact_checker[i] = HIGH;
      } else {
        tact_checker[i] = LOW;
      }
    }
    if (tact_checker[i] == HIGH) {
      digitalWrite(LED[i], HIGH);
    } else {
      digitalWrite(LED[i], LOW);
    }
    pre_tact_state[i] = tact_state;
  }
}

void debug() {
  if (tact_checker[0] == HIGH) {
    moter_direction_A(HIGH);
    moter_direction_B(HIGH);
    moter_direction_C(HIGH);
    moter_direction_D(HIGH);


    if (millis() - debug_timer < 1000) {
      for (int i = 0; i < 4; i++) {
        moter_enc_list[i] = 0;
      }
    }else if (millis() - debug_timer < 2000) {
      analogWrite(pwm_a, 200);
    }else if(millis() - debug_timer < 3000){
      analogWrite(pwm_b, 200);
      analogWrite(pwm_a, 0);
    }else if (millis() - debug_timer < 4000) {
      analogWrite(pwm_c, 200);
      analogWrite(pwm_b, 0);
    }else if( millis() - debug_timer < 5000) {
      analogWrite(pwm_d, 200);
      analogWrite(pwm_c, 0);
    }else{
      SerialUSB.println("encoder check");
      SerialUSB.println("The normal value : 1200");
      SerialUSB.print("A:");
      SerialUSB.print(moter_enc_list[0]);
      SerialUSB.print(" B:");
      SerialUSB.print(moter_enc_list[1]);
      SerialUSB.print(" C:");
      SerialUSB.print(moter_enc_list[2]);
      SerialUSB.print(" D:");
      SerialUSB.println(moter_enc_list[3]);
      Serial.println("debug finish");
      tact_checker[0] = LOW;
    }
  } else {
    debug_timer = millis();
  }
}

void loop() {
  if (Serial1.available()) {
    //接続確認用（このif文は消さないで）
    parseCtlState();
  }

  confirmation();
  if (tact_checker[0] == LOW) {
    controller_move();
    controller_spin();
    injection();
  }

  /*
  if (moter_move_check != 0) {
    SerialUSB.print("A:");
    SerialUSB.print(moter_power_list[0]);
    SerialUSB.print(" B:");
    SerialUSB.print(moter_power_list[1]);
    SerialUSB.print(" C:");
    SerialUSB.print(moter_power_list[2]);
    SerialUSB.print(" D:");
    SerialUSB.println(moter_power_list[3]);
    moter_move_check = 0;
  }
  */
}