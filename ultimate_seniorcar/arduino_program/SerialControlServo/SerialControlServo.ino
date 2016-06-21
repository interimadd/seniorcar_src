//include
#include <stdio.h>
#include <stdlib.h>
#include <mcp_can.h>
#include <SPI.h>

//define pin name
#define PWM_PORT      2
#define DIR_PORT      3
#define INT8U unsigned char
#define INT32U unsigned long

//Contoll Values
char  g_header = 'N';
float g_difference = 0;

//Contant Values
const float P_GAIN = 0.5 ;
const float MIN_DIF = 1.0 ;
const float MIN_INPUT_VOL = 5.0 ;


void setup() {
  Serial.begin(9600);
  pinMode(PWM_PORT,OUTPUT);
  pinMode(DIR_PORT,OUTPUT); 
}


// シリアル通信受信時の割り込み
// 偏差の情報を更新
void serialEvent(){

  char tmp_str;
  char header;
  char value[2];

  if( Serial.available() >= sizeof('L40')){
    while(Serial.available()>0){
      tmp_str = Serial.read();
      if(tmp_str == 'R' || tmp_str == 'L'){
        g_header = tmp_str;
        value[0] = Serial.read();
        value[1] = Serial.read();
      }
    }
  }

  g_difference = atoi(value);

}


void loop() {

  float input_value = 0;
  float output = 0;
  int   isPWM = LOW;

  if( g_difference > MIN_DIF){
    input_value = constrain( g_difference * P_GAIN , MIN_INPUT_VOL ,48); 
  }

  switch(g_header){
    case 'R':
      output = ( 0.5 - input_value / 48.0 ) * 255;
      isPWM = HIGH;
      break;
    case 'L':
      output = ( 0.5 + input_value / 48.0 ) * 255;
      isPWM = HIGH;
      break;
    default:
      output = 127;
      isPWM = LOW;
      break;
  }

  digitalWrite(PWM_PORT,isPWM);
  analogWrite(DIR_PORT,output);
  delay(20);

}
