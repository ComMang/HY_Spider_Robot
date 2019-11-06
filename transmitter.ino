// Uno
#include <SoftwareSerial.h>
#include <math.h>

#define BT_RXD 8
#define BT_TXD 7
#define JOY_SW 2

#define JOY_X 1
#define JOY_Y 0

#define STEADY 0
#define FORWARD 1
#define BACKWARD 2
#define R_ROT 3
#define L_ROT 4
#define TOGGLE 5 

#define START 1
#define STOP 2

// Constant var
const int Max_X = 690;
const int Max_Y = 690;
const int Mergin = 30;
const double pi = 3.141592;

// Output var
int pos_X = 0;
int pos_Y = 0;
bool sw_toggle = false;

// Input var
bool isStart = false;

// Serial
SoftwareSerial bluetooth(BT_RXD, BT_TXD);

// return control using pos X, Y
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
  pinMode(JOY_SW, INPUT);
  digitalWrite(JOY_SW, HIGH);
  
  bluetooth.begin(9600);
  
  Serial.println("Start.");
}
 
void loop(){
  bool isTest = true;
  // receive start/stop signal
  if(bluetooth.available()) {
    int command = bluetooth.parseInt();
    switch(command) {
      case START:
      Serial.println("Start.");
      isStart = true;
      break;
      
      case STOP:
      Serial.println("Stop.");
      isStart = false;
      break;
      
      default:
      break;
    }
  }

  // transmitt joystick control
  if(isStart|| isTest) {
    pos_X = analogRead(JOY_Y);
    pos_Y = analogRead(JOY_X);
    
    if(!digitalRead(JOY_SW)) {
      bluetooth.println(TOGGLE);
      Serial.println("press SW");
    }
    else {
      bluetooth.println(Joy_control(pos_X, pos_Y));
  
      Serial.print(pos_X);
      Serial.print(" ");
      Serial.print(pos_Y);
      Serial.print(" ");
      Serial.println(Joy_control(pos_X, pos_Y));
    }
  }
  delay(100); // delay 200ms
}
