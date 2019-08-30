/* Mega(Robot part) */

/* Library */
#include <Servo.h>
#include <SoftwareSerial.h>
#include <stdlib.h>

/* Define, Macro */
// Bluetooth module receive, transmit pin using Serial3
//#define BTRXD 15
//#define BTTXD 14

// Angel delay : 1 degree per 20ms
#define DELAY 20

// Angle No
#define END 0 // 3rd angle
#define MID 1 // 2nd angle
#define FIRST 2 // 1st angle(Main side)

// Leg No
#define NW 0
#define NE 1
#define SE 2
#define SW 3

// Send
#define START 1
#define STOP 2

// Command
#define STEADY 0
#define FORWARD 1
#define BACKWARD 2
#define R_ROT 3
#define L_ROT 4
#define END 5

// Command mode
#define SET 0 // init Servo
#define CHANGEINTO 1 // change into Servo with limited speed
#define CHANGEDIFF 2 // change difference Servo with limited speed

// Command result
#define SERVO_ERROR -999

// Steady Angle
#define STEADY_END 0
#define STEADY_MID 120
#define STEADY_FIR 90

// Initial Angle
#define INITIAL_END 0
#define INITIAL_MID 90
#define INITIAL_FIR 90

// PWM Pin
//#define getPin(legNo, angleNo) (legNo*3 + angleNo + 2)
inline int getPin(int legNo, int angleNo){ return legNo*3 + angleNo + 2; }

/* Struct */
// Command struct
typedef struct Command {
 int pin;
 int angle;
 int mode;
 struct Command *prev = NULL;
 struct Command *next = NULL; 
}Command;

// Command list struct
typedef struct CommandList {
  int listCount;
  struct Command *head = NULL;
  struct Command *tail = NULL;
}CommandList;

// Servo struct
typedef struct ServoList {
  int angle;
  Servo hServo;
}ServoList;

/* Function */
// Servo Control function
bool setServo(ServoList *target, int setAngle); // init Servo to specific angle
int changeintoServo(ServoList *target, int changeintoAngle); // change Servo angle to specific angle
int changediffServo(ServoList *target, int changediffAngle); // change Serovo angle with difference

// Servo State fuction
int* isSteadyState(ServoList *motor); // check servo is steady state, return null(all steady) or pin list(not steady Servo)

// Command and Command list function
void initCommand(CommandList *_list); // init command list sturct
void deleteCommand(Command *lastCommand); // delete command linked list (backward)
void AddCommand(CommandList *_list, int legNo, int angleNo, int mode, int angle); // add command to command list
void deleteCommandList(CommandList *_list); // delete command list sturct
bool DoCommand(CommandList *_list, ServoList *motor); // do command in command list (head to tail; FIFO)
bool DoFixPos(Command *error_command, ServoList *motor, int nCommand); // do restore command in command list(tail to head)

// Phase function
void endServo(ServoList *motor, CommandList *_list); // End phase(retrun servo to initial state)

// Thread function
void runServo(void* arg);
void mainLoop(void* arg);

/* Global Variable */
ServoList Motor[12]; // Servo handle
CommandList *list = new CommandList; // command list

int command = STEADY; // received command from bluetooth communication
bool error = false; // if not finished, goto STEADY state

/* Arduino Init */
void setup() {
  // set Serial communication bitrate
  Serial.begin(9600);
  Serial.println("Start.");
  
  // init servo(attach Servo to PWM pin)
  for(int i = 0; i < 12 ; i++) {
      Motor[i].angle = 0;
      Motor[i].hServo.attach(i + 2);
      
      if(!Motor[i].hServo.attached()) {
        Serial.print(i+2);
        Serial.println(" pin is not attached.");
        while(1);
      }
  }
  Serial.println("Servo attached.");

  // init servo angle
  for(int legNo = 0 ; legNo < 4 ; legNo++) {
    /*
    // set initial angle
    Motor[getPin(legNo, END)-2].angle = INITIAL_END;
    Motor[getPin(legNo, MID)-2].angle = INITIAL_MID;
    Motor[getPin(legNo, FIRST)-2].angle = INITIAL_FIR;
  
    // change Servo angle to Steady state
    changeintoServo(&Motor[getPin(legNo, END)-2], STEADY_END);
    changeintoServo(&Motor[getPin(legNo, MID)-2], STEADY_MID);
    changeintoServo(&Motor[getPin(legNo, FIRST)-2], STEADY_FIR);
    */
    
    // adjust Servo angle  
    setServo(&Motor[getPin(legNo, END)-2], STEADY_END); 
    setServo(&Motor[getPin(legNo, MID)-2], STEADY_MID); 
    setServo(&Motor[getPin(legNo, FIRST)-2], STEADY_FIR); 
    
    Serial.print("Set Servo angle...");
    Serial.println(legNo);
  }
  // set Bluetooth communication bitrate
  Serial3.begin(9600);
  Serial3.println(START);
  
  Serial.println("Init Servo.");

  // init command list
  initCommand(list);
  Serial.println("Init Commandlist.");
  
  // ready to move
  Serial.println("Ready.");
}

void loop() {
  // receive BT
  char btBuffer[255] = {0,};
  int btByte = 0;
  
  if(Serial3.available()) {
    Serial.print("BT> ");
    
    do { // receive byte in bluetooth
    btBuffer[btByte] = Serial3.read();
    Serial.print((int)btBuffer[btByte]);
    Serial.print(" ");
    }while(btBuffer[btByte++] != -1);
    Serial.println();

    do { // choose last received byte (except CR LT -1)
      if(btBuffer[btByte] >= 48 && btBuffer[btByte] <= 57) {
        command = (int)btBuffer[btByte] - 48; 
        break;
      }
      btByte -= 1;
    }while(btByte >= 0);
 
    if(command != STEADY){ // if runServo is finished and steady state
      Serial.print("Get> ");
      Serial.println(command);
      
      Serial3.println(STOP); // Stop transmit joystick result
      
      switch(command) // set command list
      {
        case FORWARD:
        // Todo : add foward command in command list
        
        // Test code
        AddCommand(list, SE, FIRST, CHANGEDIFF, 10);
        AddCommand(list, SE, FIRST, CHANGEDIFF, -10);
        break;
      
        case BACKWARD:
        // Todo : add backward command in command list
        
        // Test code
        AddCommand(list, SE, MID, CHANGEDIFF, 10);
        AddCommand(list, SE, MID, CHANGEDIFF, -10);
        break;
      
        case R_ROT:
        // Todo : add right rotation command in command list
        
        // Test code
        AddCommand(list, SE, END, CHANGEDIFF, 10);
        AddCommand(list, SE, END, CHANGEDIFF, -10);
        break;
      
        case L_ROT:
        // Todo : add left rotation command in command list
        
        // Test code
        AddCommand(list, SE, FIRST, CHANGEINTO, 90);
        AddCommand(list, SE, MID, CHANGEINTO, 90);
        AddCommand(list, SE, END, CHANGEINTO, 90);
        break;
    
        case END:
        endServo(Motor, list);
        break;
  
        default:
        Serial.println("Error in bluetooth communication.");
        endServo(Motor, list);
        break;
      }
      Serial.println("Run Servo.");
      
      if(!DoCommand(list, Motor)) // run Servo with command list
        Serial.println("Cannot finish command.");
      else
        Serial.println("Finish Servo.");
        
      deleteCommandList(list);
      
      Serial3.println(START); // Start transmit joystick result
    }
    else { // if steady state
      Serial.print("Cmd > ");
      Serial.println(command);
      
      int *arrPin = isSteadyState(Motor), index = 0;
      if(arrPin != NULL) {
        while(arrPin[index] != 0) {
          switch(arrPin[index++]) {
            case 2:
            AddCommand(list, NW, END, CHANGEINTO, STEADY_END);
            break;
            case 3:
            AddCommand(list, NE, END, CHANGEINTO, STEADY_END); 
            break;
            case 4:
            AddCommand(list, SE, END, CHANGEINTO, STEADY_END);
            break;
            case 5:
            AddCommand(list, SW, END, CHANGEINTO, STEADY_END);
            break;
            
            case 6:
            AddCommand(list, NW, MID, CHANGEINTO, STEADY_MID);
            break;
            case 7:
            AddCommand(list, NE, MID, CHANGEINTO, STEADY_MID); 
            break;
            case 8:
            AddCommand(list, SE, MID, CHANGEINTO, STEADY_MID);
            break;
            case 9:
            AddCommand(list, SW, MID, CHANGEINTO, STEADY_MID);
            break;

            case 10:
            AddCommand(list, NW, FIRST, CHANGEINTO, STEADY_FIR);
            break;
            case 11:
            AddCommand(list, NE, FIRST, CHANGEINTO, STEADY_FIR); 
            break;
            case 12:
            AddCommand(list, SE, FIRST, CHANGEINTO, STEADY_FIR);
            break;
            case 13:
            AddCommand(list, SW, FIRST, CHANGEINTO, STEADY_FIR);
            break;
          }
        
          if(!DoCommand(list, Motor)) // run Servo with command list
            Serial.println("Cannot finish command.");
          else
            Serial.println("Adjust Servo in steady state.");
            
          deleteCommandList(list);
        }
      }
    }
  }
  delay(100); // delay 100ms
}

/* Phase Function */
void endServo(ServoList *motor, CommandList *_list) {
  // Stop transmit joystick result
  Serial3.println(STOP);
  Serial3.end();
  
  Serial.println("Please hold robot.");
  Serial.println("Can remove VCC, GND wire from Power Supply."); 

  deleteCommandList(_list);
  delete(_list);
  
  while(1);
}

/* Servo Control Function */
// init Servo to specific angle
bool setServo(ServoList *target, int setAngle){
  if(target->hServo.attached()){
    target->angle = setAngle;
    target->hServo.write(target->angle);
    return true;
  }
  else {
    return false;
  }
}

// change Servo angle to specific angle
int changeintoServo(ServoList *target, int changeintoAngle) {
  if(changeintoAngle >= 180 || changeintoAngle < 0) // out of range
    return SERVO_ERROR;
  
  if(target->hServo.attached()){
    int prevAngle = target->angle;
    if(changeintoAngle > prevAngle) {
      for(; target->angle <= changeintoAngle ; target->angle++) { // increase angle with delay
        target->hServo.write(target->angle);
        delay(DELAY);// 1 degree / DELAY
      }
    }
    else {
      for(; target->angle >= changeintoAngle ; target->angle--) { // decrease angle with delay
        target->hServo.write(target->angle);
        delay(DELAY); // 1 degree / DELAY
      }
    }
    return changeintoAngle - prevAngle;
  }
  else {
    return SERVO_ERROR;
  }
}

// change Serovo angle with difference
int changediffServo(ServoList *target, int changediffAngle) {
  if(changediffAngle + target->angle > 180 || changediffAngle + target->angle < 0) // out of range
    return 0;
    
  if(target->hServo.attached()){
    for(int angle = 0; angle < changediffAngle ; angle++) { // change with delay
      target->angle++;
      target->hServo.write(target->angle);
      delay(DELAY); // 1 degree / DELAY
    }
    return changediffAngle;
  }
  else {
    return SERVO_ERROR;
  }
}

/* Servo State Function */
int* isSteadyState(ServoList *motor) {
  int pinList[12] = {0,}, index = 0;
  for(int legNo = 0 ; legNo < 4 ; legNo++) {
    if(motor[getPin(legNo,FIRST)-2].angle != STEADY_FIR) {
      pinList[index++] = getPin(legNo,FIRST);
    }
    if(motor[getPin(legNo,MID)-2].angle != STEADY_MID) {
      pinList[index++] = getPin(legNo,MID);
    }
    if(motor[getPin(legNo,END)-2].angle != STEADY_END) {
      pinList[index++] = getPin(legNo,END);
    }
  }
  if(pinList[0] != 0) {
    return pinList;
  }
  else {
    return NULL;
  }
}

/* Command Function */
// init command list sturct
void initCommand(CommandList *_list) { 
  _list->listCount = 0;
  _list->head = NULL;
}

// delete command list sturct
void deleteCommandList(CommandList *_list) {
  deleteCommand(_list->tail);
  initCommand(_list);
}

// delete command linked list (backward)
void deleteCommand(Command *lastCommand) {
  Command *target = lastCommand;
  while(target->prev == NULL){
    delete(target);
    target = target->prev;
  }
  delete(lastCommand);
}

// add command to command list
void AddCommand(CommandList *_list, int legNo, int angleNo, int mode, int angle) {
  Serial.println("add command.");
  
  Command *target = new Command;
  target->pin = getPin(legNo, angleNo); // target pin
  target->mode = mode; // target mode : set angle, change into specipic angle, change with difference
  target->angle = angle; // target angle after moving or delta angle
  
  if(list->listCount == 0){ // if no command in command list
    _list->head = target;
    _list->tail = target;
  }
  else { // if command exist in command list
    Command *temp = _list->tail;
    
    temp->next = target;
    target->prev = temp;
    _list->tail = target;
  }
  
  _list->listCount++;
}

// do command in command list (head to tail; FIFO)
// comment part is recovery part(if error occur during executing, do command backward)
bool DoCommand(CommandList *_list, ServoList *motor) {
  int result, count = 0;
  Command *target = _list->head;
  
  while(target != NULL){
    switch(target->mode) {
      case SET: // change Servo angle to specific angle
        result = setServo(&motor[target->pin - 2], target->angle);
      break;
      
      case CHANGEINTO: // change Servo angle to specific angle
        result = changeintoServo(&motor[target->pin - 2], target->angle);
        target->mode = CHANGEDIFF;
        target->angle = -result; 
      break;
      
      case CHANGEDIFF: // change Serovo angle with difference
        result = changediffServo(&motor[target->pin - 2], target->angle);
        target->angle = -result;
      break;
    }

    if(result != SERVO_ERROR) {
      Serial.println("complete command.");
      count++;
    }
    else {
      Serial.println("Error during command.");
      break;
    }
    
    target = target->next;
  }
  
  if(count>=_list->listCount) { // if command is done
    Serial.println("command done.");
    return true;
  }
  else { // if not
    if(!DoFixPos(target, motor, count)) {
      Serial.println("Error occur during restore state.");
      endServo(motor, _list);
    }
    return false;
  }
}

bool DoFixPos(Command *error_command, ServoList *motor, int nCommand) {
  int result, count = 0;
  Command *target = error_command;

  while(target != NULL){
    switch(target->mode) {
      case SET: // change Servo angle to specific angle
        result = setServo(&motor[target->pin - 2], target->angle);
      break;
      
      case CHANGEINTO: // change Servo angle to specific angle
        result = changeintoServo(&motor[target->pin - 2], target->angle);
      break;
      
      case CHANGEDIFF: // change Serovo angle with difference
        result = changediffServo(&motor[target->pin - 2], target->angle);
      break;
    }

    if(result != SERVO_ERROR) {
      Serial.println("complete during fixing Pos.");
      count++;
    }
    else {
      Serial.println("Error during fixing Pos.");
      break;
    }
    
    target = target->prev;
  }
  
  if(count >= nCommand) {// if command is done
    Serial.println("fixing Pos done.");
    return true;
  }
  else // if not
    return false;
}
