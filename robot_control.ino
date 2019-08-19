/* Mega(Robot part) */

/* Library */
#include <Servo.h>
#include <SoftwareSerial.h>
#include <FreeRTOS_AVR.h> // install FreeRTOS library

/* Define, Macro */
// Bluetooth module receive, transmit pin
#define BTRXD 8
#define BTTXD 7

// Angel delay : 1 degree per 25ms
#define DELAY 25

// Angle No
#define END 0 // 3rd angle
#define MID 1 // 2nd angle
#define FIRST 2 // 1st angle(Main side)

// Leg No
#define NW 0
#define NE 1
#define SE 2
#define SW 3

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
#define STEADY_END 180
#define STEADY_MID 110
#define STEADY_FIR 90

// Initial Angle
#define INITIAL_END 90
#define INITIAL_MID 180
#define INITIAL_FIR 90

// PWM Pin
inline int getPin(int legNo, int angleNo){ return legNo*3 + angleNo + 2; }

/* Struct */
// Command struct
typedef struct Command {
 int pin;
 int angle;
 int mode;
 struct Command *prev;
 struct Command *next; 
}Command;

// Command list struct
typedef struct CommandList {
  int listCount;
  struct Command *head;
  struct Command *tail;
}CommandList;

// Servo struct
typedef struct ServoList {
  int angle;
  Servo hServo;
}ServoList;

/* Global Variable */
//SoftwareSerial bluetooth(BT_RXD, BT_TXD); // set bluetooth communication serial

ServoList Motor[12]; // Servo handle
CommandList *list = new CommandList; // command list
int command = STEADY; // received command from bluetooth communication
bool error = false; // if not finished, goto STEADY state
bool isfinish = false; // thread finish

SemaphoreHandle_t hSem = NULL; // semaphore handle

/* Function */
// Servo Control function
bool setServo(ServoList *target, int setAngle); // init Servo to specific angle
int changeintoServo(ServoList *target, int changeintoAngle); // change Servo angle to specific angle
int changediffServo(ServoList *target, int changediffAngle); // change Serovo angle with difference

// Servo State fuction
int* isSteadyState(ServoList *motor); // check servo is steady state, return null(all steady) or pin list(not steady Servo)

// Command and Command list function
void initCommand(CommandList *list); // init command list sturct
void deleteCommand(Command *lastCommand); // delete command linked list (backward)
void AddCommand(CommandList *list, int legNo, int angleNo, int mode, int angle); // add command to command list
void deleteCommandList(CommandList *list); // delete command list sturct
bool DoCommand(CommandList *list, ServoList *motor); // do command in command list (head to tail; FIFO)
bool DoCommand(Command *error_command, ServoList *motor, int nCommand); // do restore command in command list(tail to head)

// Phase function
void endServo(); // End phase(retrun servo to initial state)

// Thread function
static void runServo(void* arg);
static void mainLoop(void* arg);

/* Thread */
static void mainLoop(void* arg) { // main loop
  while(1) {
    /*
    if(bluetooh.available()) { // if received command is avilable by bluetooth
      
      // Todo : bluetooth module(filltered), output : command 
      
      // Todo : if no error in previous or bluetooth data is available
      
    }

    if(!isfinish) { // if runServo is not finished
      
      // Todo : Save received command or ignore received command
      
    }
    else if(isSteadyState(Motor) != NULL && command == STEADY){ // if runServo is finished and not steady state
      
      // Todo : return steady state in any condition
      
      xSemaphoreGive(hSem); // give Semphore to runServo
    }
    else { // if runServo is finished and steady state
      switch(command) // set command list
      {
        case FORWARD:
        // Todo : add foward command in command list
        break;
      
        case BACKWARD:
        // Todo : add backward command in command list
        break;
      
        case R_ROT:
        // Todo : add right rotation command in command list
        break;
      
        case L_ROT:
        // Todo : add left rotation command in command list
        break;
    
        case END:
        endServo();
        break;
      }
      
      xSemaphoreGive(hSem); // give Semphore to runServo
    }
    
    vTaskDelay((100L * configTICK_RATE_HZ) / 1000L); // delay 100ms
    */

    // test code
    int angle, data;
    if(Serial.available() && isfinish) {
      Serial.println("Servo start.");
      
      data = Serial.parseInt();
      Serial.println(data);
      angle = data % 100;
    
      switch((int)(data/100)) {
        case 1:
        AddCommand(list, SE, FIRST, CHANGEDIFF, angle/2);
        AddCommand(list, SE, FIRST, CHANGEDIFF, angle/2);
        break;
        
        case 2:
        AddCommand(list, SE, FIRST, CHANGEDIFF, -angle);
        break;
        
        case 3:
        AddCommand(list, SE, MID, CHANGEDIFF, angle/2);
        AddCommand(list, SE, MID, CHANGEDIFF, angle/2);
        break;
        
        case 4:
        AddCommand(list, SE, MID, CHANGEDIFF, -angle/2);
        AddCommand(list, SE, MID, CHANGEDIFF, -angle/2);
        break;
        
        case 5:
        AddCommand(list, SE, END, CHANGEDIFF, angle/4);
        AddCommand(list, SE, END, CHANGEDIFF, angle/4);
        AddCommand(list, SE, END, CHANGEDIFF, angle/4);
        AddCommand(list, SE, END, CHANGEDIFF, angle/4);
        break;
        
        case 6:
        AddCommand(list, SE, END, CHANGEDIFF, -angle);
        break;
        
        case 7:
        AddCommand(list, SE, FIRST, CHANGEINTO, angle);
        break;
        
        case 8:
        AddCommand(list, SE, MID, CHANGEINTO, angle);
        break;
        
        case 9:
        AddCommand(list, SE, END, CHANGEINTO, angle);
        break;
        
        default:
        Serial.println("Out of range.");
        break;
      }
      xSemaphoreGive(hSem); // give Semphore to runServo
    } 
    vTaskDelay((100L * configTICK_RATE_HZ) / 1000L); // delay 100ms
  }
}

static void runServo(void* arg) { // run Servo with command list
  while(1) {
    xSemaphoreTake(hSem, portMAX_DELAY); // wait until taking Samaphore from mainLoop, parameter : semphore handle, delay 

    isfinish = false; // not finished
    
    if(!DoCommand(list, Motor)) // run Servo with command list
      Serial.println("Cannot finish command.");
    else
      Serial.println("Finish Servo.");
    
    command = STEADY; // reset command
    isfinish = true; // finished
  }
}

/* Arduino Init */
void setup() {
  // set Serial communication bitrate
  Serial.begin(9600);

  // set bluetooth communication bitrate
//  bluetooth.begin(9600);

  // init servo(attach Servo to PWM pin)
  for(int i = 0; i < 12 ; i++) {
      Motor[i].angle = 0;
      Motor[i].hServo.attach(i + 2);
  }

  // init servo angle
  for(int legNo = 0 ; legNo < 4 ; legNo++) {
    // set initial angle
    Motor[getPin(legNo, END)-2].angle = INITIAL_END;
    Motor[getPin(legNo, MID)-2].angle = INITIAL_MID;
    Motor[getPin(legNo, FIRST)-2].angle = INITIAL_FIR;
  
    // change Servo angle to Steady state
    changeintoServo(&Motor[getPin(legNo, END)-2], STEADY_END);
    changeintoServo(&Motor[getPin(legNo, MID)-2], STEADY_MID);
    changeintoServo(&Motor[getPin(legNo, FIRST)-2], STEADY_FIR);
  
    // adjust Servo angle  
    setServo(&Motor[getPin(legNo, END)-2], STEADY_END); 
    setServo(&Motor[getPin(legNo, MID)-2], STEADY_MID); 
    setServo(&Motor[getPin(legNo, FIRST)-2], STEADY_FIR); 
  }
  
  // init Semaphore
  hSem = xSemaphoreCreateCounting(1,0); // parameter : max thread num, initial thread num

  // create thread(main Loop, run Servo)
  portBASE_TYPE Thread1, Thread2;
  
  Thread1 = xTaskCreate(mainLoop, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL); // parameter : callback func, name, stack depth, parameter, priority(low : 1), handle of create task 
  Thread2 = xTaskCreate(runServo, NULL, configMINIMAL_STACK_SIZE, NULL, 2, NULL);

  // check error during semaphore or thread creation
  if(hSem == NULL || Thread1 != pdPASS || Thread2 != pdPASS) {
    Serial.println("Creation prob.");
    
    while(1);
  }
  
  Serial.println("Ready.");

  // start Scheduler
  vTaskStartScheduler();

  // if error during task
  Serial.println("Insufficient RAM");
  while(1);
}

void loop() {
  // not use 
}

/* Phase Function */
void endServo() {
  Serial.println("Please hold robot.");
        
  vTaskDelay((1000L * configTICK_RATE_HZ) / 1000L); // delay 1000ms

  for(int legNo = 0 ; legNo < 4 ; legNo++) { // change Servo angle to initial state
    changeintoServo(&Motor[getPin(legNo, END)-2], INITIAL_END);
    changeintoServo(&Motor[getPin(legNo, MID)-2], INITIAL_MID);
    changeintoServo(&Motor[getPin(legNo, FIRST)-2], INITIAL_FIR);
  }
  
  Serial.println("Can remove VCC, GND wire from Power Supply.");

  deleteCommandList(list);
  delete(list);
  
  while(1);
}

/* Servo Function */
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
        
        if(hSem == NULL) // 1 degree / DELAY
          delay(DELAY);
        else
          vTaskDelay((DELAY * configTICK_RATE_HZ) / 1000L); 
      }
    }
    else {
      for(; target->angle >= changeintoAngle ; target->angle--) { // decrease angle with delay
        target->hServo.write(target->angle);
        
        if(hSem == NULL) // 1 degree / DELAY
          delay(DELAY);
        else
          vTaskDelay((DELAY * configTICK_RATE_HZ) / 1000L); 
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
      vTaskDelay((DELAY * configTICK_RATE_HZ) / 1000L); // 1 degree / DELAY
    }
    return changediffAngle;
  }
  else {
    return SERVO_ERROR;
  }
}

/* State Function */
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
void initCommand(CommandList *list) { 
  list->listCount = 0;
  list->head = NULL;
}

// delete command list sturct
void deleteCommandList(CommandList *list) {
  deleteCommand(list->tail);
  
  list->head = NULL;
  list->tail = NULL;
  list->listCount = 0;
}

// delete command linked list (backward)
void deleteCommand(Command *lastCommand) {
  Command *target = lastCommand;
  while(1){
    target = target -> prev;
    delete(target->next);
    
    target->next = NULL;
    
    if(target->prev == NULL) break;
  }
}

// add command to command list
void AddCommand(CommandList *list, int legNo, int angleNo, int mode, int angle) {
  Command *target = new Command;
  
  target->pin = getPin(legNo, angleNo); // target pin
  target->mode = mode; // target mode : set angle, change into specipic angle, change with difference
  target->angle = angle; // target angle after moving or delta angle
  target->next = NULL;
  
  if(list->listCount == 0){ // if no command in command list
    list->head = target;
    list->tail = target;
    
    target->prev = NULL;
  }
  else { // if command exist in command list
    Command *temp = list->tail;
    
    list->tail->next = target;
    list->tail = target;
    target->prev = temp;
  }
  
  list->listCount++;
}

// do command in command list (head to tail; FIFO)
// comment part is recovery part(if error occur during executing, do command backward)
bool DoCommand(CommandList *list, ServoList *motor) {
  int result, count = 0;
  Command *target = list->head;
  
  while(1){
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

    if(result != SERVO_ERROR)
      count++;
    else
      break;
    
    if(target->next == NULL) // if last command
      break;
    else // if not, go next command
      target = target->next;
  }
  
  if(count == list->listCount) { // if command is done
    deleteCommandList(list);
    return true;
  }
  else { // if not
    if(!DoCommand(target, motor, count)) {
      Serial.println("Error occur during restore state.");
      deleteCommandList(list);
      endServo();
    }
    return false;
  }
}

bool DoCommand(Command *error_command, ServoList *motor, int nCommand) {
  int result, count = 0;
  Command *target = error_command;
  
  while(1){
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

    if(result != SERVO_ERROR)
      count++;
    else
      break;
    
    if(target->prev == NULL) // if last command
      break;
    else // if not, go next command
      target = target->prev;
  }
  
  if(count == nCommand)// if command is done
    return true;
  else // if not
    return false;
}
