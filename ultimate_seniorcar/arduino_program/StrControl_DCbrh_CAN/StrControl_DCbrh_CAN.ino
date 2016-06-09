//include
#include <stdio.h>
#include <stdlib.h>
#include <mcp_can.h>
#include <SPI.h>

//define pin name
#define pwm      2
#define dir      3
#define INT8U unsigned char
#define INT32U unsigned long

//Global variables
float P_gain =0.5;
float MIN_VOL = 5.0;
float duty = 0.5;
float input_voltage = 0;
char header[1];
float Set_point = 5;
float output = 128;
float R_Value = 0;
float L_Value = 0;
int min_error = 1;
char inputValue[2];
INT8U len = 0;
INT8U buf[8];
INT8U flag[1];  
boolean getdata[4]={false};
char readAll[256]; // 受信している全てのバッファを格納するための変数

  
void setup() {
  Serial.begin(9600);
  CAN.begin(CAN_500KBPS); 
  pinMode(pwm,OUTPUT);
  pinMode(dir,OUTPUT);
  digitalWrite(pwm,LOW);
}

void loop() {

  if(flag[0] <= 0){
    //CANで現在のフラグを取得
    digitalWrite(pwm,LOW);
     if(CAN.checkReceive() == CAN_MSGAVAIL)                   // データが届いたかチェック
    {
      INT32U id = CAN.getCanId();
      switch(id){           
            case 0x15:
            flag[0] = buf[7];  // 
            getdata[1] = true;
            break;
        }
      CAN.readMsgBuf(&len, buf);       
        }
  }

  else{

//シリアル信号でステアリング角の偏差を取得
  if ( Serial.available() >= sizeof('L40')) {

    // 全て読み込んでから、末尾の指令値を取り出すようにする
    int bufferNum = 0;
    int headerIndex = 0;
    while(Serial.available()>0){
      readAll[bufferNum] = Serial.read();
      if(readAll[bufferNum] == 'R' || readAll[bufferNum] == 'L'){
        headerIndex = bufferNum;
      }
      bufferNum++;
    }

    // 最後の指令値だと三桁全て読み込んでる保証がないので一つ前のものを指令値とする
    headerIndex--;
    if(headerIndex < 0){
      headerIndex = 0;
    }
    //delay(20);

    header[0]     = readAll[headerIndex];
    inputValue[0] = readAll[headerIndex + 1];
    inputValue[1] = readAll[headerIndex + 2];

    if( header[0] == 'R'){
      R_Value = atoi(inputValue);
      R_Value = constrain(R_Value, 0, 48);
      L_Value = 0;
        
      if(R_Value > Set_point){  
        digitalWrite(pwm,HIGH);
        input_voltage = R_Value * P_gain;
        input_voltage = constrain(input_voltage, MIN_VOL, 24);
        duty = 0.5 - input_voltage / 48;
        output = duty * 255;
      }

      else if(R_Value <= Set_point && R_Value > min_error ){
        digitalWrite(pwm,HIGH);
        input_voltage = MIN_VOL;
        duty = 0.5 - input_voltage / 48;
        output = duty * 255;
      }
    
      else if(R_Value <= min_error){
        digitalWrite(pwm,LOW);
      }
    }
 
    else if( header[0] ==  'L'){
      L_Value = atoi(inputValue);
      L_Value = constrain(L_Value, 0, 48);
      R_Value = 0;
        
      if(L_Value > Set_point){  
        digitalWrite(pwm,HIGH);
        input_voltage = L_Value * P_gain;
        input_voltage = constrain(input_voltage, MIN_VOL, 24);
        duty = 0.5 + input_voltage / 48;
        output = duty * 255;
      }

      else if(L_Value <= Set_point && L_Value > min_error ){
        digitalWrite(pwm,HIGH);
        input_voltage = MIN_VOL;
        duty = 0.5 + input_voltage / 48;
        output = duty * 255;
      }
     
      else if(L_Value <= min_error){
        digitalWrite(pwm,LOW);
      }
    }

  }
  
  }
  analogWrite(dir,output); 
  delay(20);
}

