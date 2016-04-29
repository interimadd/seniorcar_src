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
  
void setup() {
  Serial.begin(9600);
  CAN.begin(CAN_500KBPS); 
  pinMode(pwm,OUTPUT);
  pinMode(dir,OUTPUT); 
}

void loop() {

//CANで現在のフラグを取得
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


  if (flag[0] > 0){

//シリアル信号でステアリング角の偏差を取得
  if ( Serial.available() >= sizeof('L40')) {
        header[0] = Serial.read();
        inputValue[0] = Serial.read();
        inputValue[1] = Serial.read();

        if( header[0] == 'R'){
        R_Value = atoi(inputValue);
        R_Value = constrain(R_Value, 0, 48);
        L_Value = 0;
        
    if(R_Value > Set_point){  
    digitalWrite(pwm,HIGH);
    input_voltage = R_Value * P_gain;
    input_voltage = constrain(input_voltage, 8, 24);
    duty = 0.5 - input_voltage / 48;
    output = duty * 255;
    }
    else if(R_Value < Set_point && R_Value > min_error ){
    digitalWrite(pwm,HIGH);
    input_voltage = 8;
    duty = 0.5 - input_voltage / 48;
    output = duty * 255;
    }
     else if(R_Value < min_error){
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
    input_voltage = constrain(input_voltage, 8, 24);
    duty = 0.5 + input_voltage / 48;
    output = duty * 255;
    }
    else if(L_Value < Set_point && L_Value > min_error ){
    digitalWrite(pwm,HIGH);
    input_voltage = 8;
    duty = 0.5 + input_voltage / 48;
    output = duty * 255;
    }
     else if(L_Value < min_error){
     digitalWrite(pwm,LOW);
     }
  }
  }
  
  }
  analogWrite(dir,output); 
  delay(2);
 }

