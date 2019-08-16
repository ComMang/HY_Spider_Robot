// Uno
#include <SoftwareSerial.h>
#include <math.h>

#define BT_RXD 8
#define BT_TXD 7
#define JOY_X 1
#define JOY_Y 0

#define STEADY 0
#define FORWARD 1
#define BACKWARD 2
#define R_ROT 3
#define L_ROT 4

const int Max_X = 1022;
const int Max_Y = 1022;
const int Mergin = 200;
const double pi = 3.141592;

int pos_X = 0;
int pos_Y = 0;
int result = STEADY;
char data;

SoftwareSerial bluetooth(BT_RXD, BT_TXD);

int Joy_control(int x, int y) {
  int result = STEADY;
  int pos_X = x - Max_X/2;
  int pos_Y = y - Max_Y/2;

  if(abs(pos_X) > Mergin || abs(pos_Y) > Mergin) {    
    double angle = atan(abs(pos_Y)/abs(pos_X));
    
    if(angle >= pi/4){
      if(pos_Y > 0)
        result = FORWARD;
      else
        result = BACKWARD;
    }
    else if(0 < angle < pi/4) {
      if(pos_X > 0)
        result = R_ROT;
      else
        result = L_ROT;
    }
  }
  
  return result;
}

void setup(){
  Serial.begin(9600);
  bluetooth.begin(9600);
  
  Serial.println("Start");
}
 
void loop(){
  Serial.flush();
  pos_X = analogRead(JOY_X);
  pos_Y = analogRead(JOY_Y);

  result = Joy_control(pos_X, pos_Y);

  while(!Serial.available());

  Serial.print("T> ");
  while(Serial.available()) {
    data = Serial.read();
    if(data == -1) break;
    bluetooth.write(data);
    Serial.print(data);
    // 시리얼 통신에서는 9600bps 기준으로
    // read 를 사용할 때 1ms 의 딜레이를 줘야 한다.
    delay(1);
  }
  Serial.print("\n");

//  bluetooth.println(result);
//  
//  Serial.print(pos_X);
//  Serial.print(" ");
//  Serial.print(pos_Y);
//  Serial.print(" ");
//  Serial.println(result);
//  
  delay(500);
}
