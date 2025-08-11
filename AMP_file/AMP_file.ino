//abcdのpwmとdirectonの数の決定
const int pwm_a = 2;       
const int direction_a = 30;
const int pwm_b = 5;
const int direction_b = 33;
const int pwm_c = 3;
const int direction_c = 31;
const int pwm_d = 6;
const int direction_d = 34;

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
int btnState[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// xは右にやると+、yは下にやると+
// -5 ~ 5(ESP32側のプログラムで変更可能)
//               lx ly rx ry
int axiState[] = {0, 0, 0, 0};

//左スティックの状態を表す変数
int lx_state = 0;
int ly_state = 0;
int rx_state = 0;

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
  Serial1.begin(115200); // ESP用
  Serial.begin(115200); // PC
}

//モーター
//それぞれのモーターの回転の向きを定義//
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


void moter_front(int on_off, int front, int moter_power){
  //前か後ろに移動

  if(on_off == 1)
  {
    analogWrite(pwm_a, moter_power);
    analogWrite(pwm_b, moter_power);
    analogWrite(pwm_c, moter_power);
    analogWrite(pwm_d, moter_power);
    if (front == 1)
    {
      moter_direction_A(HIGH);
      moter_direction_B(LOW);
      moter_direction_C(LOW);
      moter_direction_D(HIGH);
    }else{
      moter_direction_A(LOW);
      moter_direction_B(HIGH);
      moter_direction_C(HIGH);
      moter_direction_D(LOW);
    }
  }  
}


void moter_right(int on_off, int front, int moter_power){
  //右か左に移動

  if(on_off == 1)
  {
    analogWrite(pwm_a, moter_power);
    analogWrite(pwm_b, moter_power);
    analogWrite(pwm_c, moter_power);
    analogWrite(pwm_d, moter_power);
    if (front == 1)
    {
      moter_direction_A(HIGH);
      moter_direction_B(HIGH);
      moter_direction_C(HIGH);
      moter_direction_D(HIGH);
    }else{
      moter_direction_A(LOW);
      moter_direction_B(LOW);
      moter_direction_C(LOW);
      moter_direction_D(LOW);
    }
  }
}


void moter_AD(int on_off,int front, int moter_power)//傾き正の向きに移動する関数//
{
  if(on_off == 1) //動かすモーターを固定//
  {
    analogWrite(pwm_a, moter_power);
    analogWrite(pwm_b,0);
    analogWrite(pwm_c,0);
    analogWrite(pwm_d, moter_power);
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


void moter_BC(int on_off,int front, int moter_power)//傾き負の向きに移動する関数//
{
  if(on_off == 1)
  {
    analogWrite(pwm_a,0);
    analogWrite(pwm_b, moter_power);
    analogWrite(pwm_c, moter_power);
    analogWrite(pwm_d,0);
    if(front == 1)
    {
      moter_direction_B(LOW);
      moter_direction_C(LOW);
    }
    else
    {
      moter_direction_B(HIGH);
      moter_direction_C(HIGH);
    } 
  }
}


void moter_spin(int on_off, int left, int moter_power)//回転する関数//
{
  if(on_off == 1)
  {
    analogWrite(pwm_a, moter_power);
    analogWrite(pwm_b, moter_power);
    analogWrite(pwm_c, moter_power);
    analogWrite(pwm_d, moter_power);
    if (left == 1)
    {
      moter_direction_A(HIGH);
      moter_direction_B(LOW);
      moter_direction_C(HIGH);
      moter_direction_D(LOW);
    }
    else
    {
      moter_direction_A(LOW);
      moter_direction_B(HIGH);
      moter_direction_C(LOW);
      moter_direction_D(HIGH);
    }
  }
}

void moter_initialization(){ 
  //モータの動きを初期化
  moter_front(0, 0);
  moter_right(0, 0);
  moter_AD(0, 0);
  moter_BC(0, 0);
  moter_spin(0, 0);
  analogWrite(pwm_a, 0);
  analogWrite(pwm_b, 0);
  analogWrite(pwm_c, 0);
  analogWrite(pwm_d, 0);
}


void wheel_check(){
  //足回りがちゃんと動くかの確認用
  int time = millis();
  if(10000  < time && 11000 > time){
    moter_front(1, 1);
  }else if(14000  < time && 15000 > time){
    moter_right(1, 1);
  }else if(18000  < time && 19000 > time){
    moter_front(1, 0);
  }else if(22000  < time && 23000 > time){
    moter_right(1, 0);
  }else if(26000  < time && 27000 > time){
    moter_AD(1, 1);
  }else if(30000  < time && 31000 > time){
    moter_AD(1, 0);
  }else if(34000  < time && 35000 > time){
    moter_BC(1, 1);
  }else if(38000  < time && 39000 > time){
    moter_BC(1, 0);
  }else if(42000  < time && 43000 > time){
    moter_spin(1, 1);
  }else if(46000  < time && 47000 > time){
    moter_spin(1, 0);
  }else{
    moter_initialization();
  }
}

//無線
int getBtnState(String key){
  // getBtnState("A")で、〇ボタンのon/offが返ってくる
  // 押されてれば1、押されてなければ0


  String keyMap[] = {"UP", "DOWN", "LEFT", "RIGHT", "A", "B", "X", "Y", "L1", "R1", "L2", "R2"};

  for(int i = 0; i < 12; i++){
    if(keyMap[i] == key){
      return btnState[i];
    }
  }

  return 0;
}


int getAxiState(String key, bool isBin = false){
  // getAxiState("LY")で、スティックの軸の正規化された値が返ってくる
  // Xは右が+、Yは下が+
  // 範囲はESPのプログラムによって変わる。-4 ~ 4？y軸については上に上がるほど低い値をとる（－4に近づくという点に注意）


  String keyMap[] = {"LX", "LY", "RX", "RY"};
  int binThreshold = 2;

  for(int i = 0; i < 4; i++){
    if(keyMap[i] == key){
      if(isBin){
        if(axiState[i] >= binThreshold){
          return 1;
        } else if(axiState[i] <= -binThreshold){
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


void parseCtlState(){
  // ESP32からの通信を解析し、コントローラーの配列を設定

  // 改行コードが来るまでバッファに入れ、改行コードを削除
  String data = Serial1.readStringUntil(0x0a);
  data.trim();

  // "DATA: "から始まらないものは解析しない
  // "INFO: Controller has disconnected"とかそういうメッセージなので、そのままPC側に送る
  if(!data.startsWith("DATA: ")){
    Serial.println(data);
    return;
  }

  data.replace("DATA: ", "");

  int len = data.length();

  // data = "0 0 1 1 0 0 0 0 1 0 0 0 3 -2 0 4"
  // コントローラの状況の文字列データを空白で切り分け、数字として仮の配列に格納
  String buf = "";
  int idx = 0;
  int ctlState[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  for(int i = 0; i < len; i++){
    char cursor = data.charAt(i);

    if(cursor == ' '){
      ctlState[idx] = buf.toInt();

      buf = "";
      idx++;
      continue;
    }

    buf.concat(cursor);
  }

  ctlState[17] = buf.toInt();

  // 仮の配列からほかでも使うグローバルの配列に格納
  for(int i = 0; i < 14; i++){
    btnState[i] = ctlState[i];
  }
  for(int i = 0; i < 4; i++){
    axiState[i] = ctlState[i + 14];
  }
}

void controller_move(){
  if((lx_state == 0 && ly_state == 0) || (lx_state != getAxiState("LX") || ly_state != getAxiState("LY")) && rx_state == 0)
  {
    moter_initialization();
  }
  lx_state = getAxiState("LX");
  ly_state = getAxiState("LY");

  //前後左右
  if(lx_state == 0)
  {
    //縦移動
    if(ly_state < 0)
    {
      moter_front(HIGH, HIGH);
    }
    if(ly_state > 0)
    {
      moter_front(HIGH, LOW);
    }
  }
  if(ly_state == 0)
  {
    //横移動
    if(lx_state > 0)
    {
      moter_right(HIGH, HIGH);
    }
    if(lx_state < 0)
    {
      moter_right(HIGH, LOW);
    }
  }
  //斜め
  if(lx_state != 0 && lx_state != 0)
  {
    if(lx_state > 0 && ly_state < 0)
    {
      moter_AD(HIGH, HIGH);
    }
    if(lx_state < 0 && ly_state > 0)
    {
      moter_AD(HIGH, LOW);
    }
    if(lx_state < 0 && ly_state < 0)
    {
      moter_BC(HIGH, HIGH);
    }
    if(lx_state > 0 && ly_state > 0)
    {
      moter_BC(HIGH, LOW);
    }
  }
}

void controller_spin(){
  if(rx_state == 0 && lx_state != 0 && ly_state != 0)
  {
    moter_initialization();
  }
  rx_state = getAxiState("RX");
  if(rx_state > 0)
  {
    moter_spin(HIGH, HIGH);
  }
  if(rx_state < 0)
  {
    moter_spin(HIGH, LOW);
  }
}

void loop() {
  if(Serial1.available()){
    //接続確認用（このif文は消さないで）
    parseCtlState();
  }
  controller_move();
  controller_spin();
}