// Mega
#include <SoftwareSerial.h>

#define BT_RXD 8
#define BT_TXD 7

String Received=""; //받는 문자열

SoftwareSerial bluetooth(BT_RXD, BT_TXD);
 
void setup() {
  Serial.begin(9600);   //시리얼모니터 
  bluetooth.begin(9600); //블루투스 시리얼 개방
    
  Serial.println("Start");
}
 
void loop() {
  while(bluetooth.available())  //mySerial에 전송된 값이 있으면
  {
    Received+=(char)bluetooth.read();   //수신되는 문자를 myString에 모두 붙임 (1바이트씩 전송되는 것을 연결)
    delay(5);           //수신 문자열 끊김 방지
  }
  if(!Received.equals(""))  //myString 값이 있다면
  {
    Serial.println("R> "+Received); //시리얼모니터에 myString값 출력
    Received="";  //myString 변수값 초기화
  }
}
