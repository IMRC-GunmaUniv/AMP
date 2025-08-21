//コントローラーの無線接続に関するコードです。以降このコードをコピーして現在使っているコード（AMP)に書き加える形で使います。
// --- internal variable ---
// 押されてると1、押されてないと0
//                u  d  l  r  a  b  x  y l1 r1 l2 r2 ls rs
int btnState[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


// xは右にやると+、yは下にやると+
// -5 ~ 5(ESP32側のプログラムで変更可能)
//               lx ly rx ry
int axiState[] = {0, 0, 0, 0};


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




void setup() {
  Serial1.begin(115200); // ESP用
  Serial.begin(115200); // PC
}


void loop() {
  if(Serial1.available()){
    //接続確認用（このif文は消さないで）
    parseCtlState();
  }
  Serial.println(getAxiState("LX"));
}