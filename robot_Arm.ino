#include <Servo.h>

//elbow BED step direction and enable pin
#define stp_elbow 3
#define dir_elbow 4
#define EN_elbow  8
//base BED step direction and enable pin
#define stp_base 9
#define dir_base 10
#define EN_base  11
//shared microstep pins
#define MS1_base 5
#define MS2_base 6
#define MS3_base 7
#define MS1_elbow 17
#define MS2_elbow 18
#define MS3_elbow 19

#define joyPin1 0 // slider variable connecetd to analog pin 0
#define joyPin2 1 // slider variable connecetd to analog pin 1
#define pushButton 16 //pin connected to pushbutton

const int stepsPerRevolution = 200;

Servo wristServo;
Servo gripperServo;

int value1 = 0;                  // variable to read the value from the analog pin 0
int value2 = 0;                  // variable to read the value from the analog pin 1
int pos = 0; // position of servo motor
//int previousReading = 0;
boolean moveForward = false;
boolean moveBackward = false;
boolean moveUp = false;
boolean moveDown = false;

volatile long previousPressTime = 0;
volatile boolean setPenInPosition = false;
char c; 

// microstep resolution
// 1. eigth step 
// 2. quarter step
// 3. half step
// 4. full step
uint8_t resolution[4][3] = {
  {1,1,0},
  {0,1,0},
  {1,0,0},
  {0,0,0}
};

void setup() {
  Serial.begin(9600);
  //setup servos 
  wristServo.attach(12);
  gripperServo.attach(13);
  wristServo.write(160);
  gripperServo.write(0);
  //stepper motor pins
  pinMode(stp_elbow, OUTPUT);
  pinMode(dir_elbow, OUTPUT);
  pinMode(stp_base, OUTPUT);
  pinMode(dir_base, OUTPUT);
  pinMode(MS1_base, OUTPUT);
  pinMode(MS2_base, OUTPUT);
  pinMode(MS3_base, OUTPUT);
  pinMode(MS1_elbow, OUTPUT);
  pinMode(MS2_elbow, OUTPUT);
  pinMode(MS3_elbow, OUTPUT);
  pinMode(EN_elbow, OUTPUT);
  pinMode(EN_base, OUTPUT);
  pinMode(pushButton, INPUT);
  resetBEDPins(); //Set step, direction, microstep and enable pins to default states
  
  attachInterrupt(0, movePen, LOW); // set interrupt for the gripper movement.
  delay(100);
}

// convert analog reading to figure we can work with
int treatValue(int data) {
  return (data * 9 / 1024) + 48;
}

void loop() { 
  switch (digitalRead(pushButton)){
    case 0:
      if(52 < treatValue(analogRead(joyPin1))){
        velocity(abs(52 - treatValue(analogRead(joyPin1)))); // get velocity
        if(!moveForward){
          digitalWrite(dir_base, LOW); //Pull direction pin low to move "forward"
          digitalWrite(EN_base, LOW);
          digitalWrite(EN_elbow, HIGH);
          clearFlags();
          moveForward = false;
        }
        digitalWrite(stp_base,HIGH); //Trigger one step forward
        delay(1);
        digitalWrite(stp_base,LOW); //Pull step pin low so it can be triggered again
        delay(1);
      }
      else if(treatValue(analogRead(joyPin1)) < 52){
        velocity(52- treatValue(analogRead(joyPin1))); // get velocity
        if(!moveBackward){
          digitalWrite(dir_base, HIGH); //Pull direction pin low to move "backward"
          digitalWrite(EN_base, LOW);
          digitalWrite(EN_elbow, HIGH);
          clearFlags();
          moveBackward = false;
        }
        digitalWrite(stp_base,HIGH); //Trigger one step forward
        delay(1);
        digitalWrite(stp_base,LOW); //Pull step pin low so it can be triggered again
        delay(1);
      }
      else if(treatValue(analogRead(joyPin2)) > 52){
        velocity(abs(52 - treatValue(analogRead(joyPin2)))); // get velocity
        if(!moveUp){
          digitalWrite(dir_elbow, LOW); //Pull direction pin low to move "forward"
          digitalWrite(EN_elbow, LOW);
          digitalWrite(EN_base, HIGH);
          clearFlags();
          moveUp = false;
        }
        digitalWrite(stp_elbow,HIGH); //Trigger one step forward
        delay(1);
        digitalWrite(stp_elbow,LOW); //Pull step pin low so it can be triggered again
        delay(1);
      }
      else if(treatValue(analogRead(joyPin2)) < 52){
        velocity(52 - treatValue(analogRead(joyPin2))); // get velocity
        if(!moveDown){
          digitalWrite(dir_elbow, HIGH); //Pull direction pin low to move "backward"
          digitalWrite(EN_elbow, LOW);
          digitalWrite(EN_base, HIGH);
          clearFlags();
          moveDown = false;
        }
        digitalWrite(stp_elbow,HIGH); //Trigger one step forward
        delay(1);
        digitalWrite(stp_elbow,LOW); //Pull step pin low so it can be triggered again
        delay(1);
      }
      else if(treatValue(analogRead(joyPin2)) == 52 && treatValue(analogRead(joyPin1)) == 52){
        resetBEDPins();
        clearFlags();
      }
      break;
    case 1:
      if(treatValue(analogRead(joyPin1)) > 52){
        velocity(abs(52 - treatValue(analogRead(joyPin1)))); // get velocity
        if(!moveForward){
          digitalWrite(dir_base, LOW); //Pull direction pin low to move "forward"
          digitalWrite(dir_elbow, HIGH); //Pull direction pin low to move "forward"
          digitalWrite(EN_base, LOW);
          digitalWrite(EN_elbow, LOW);
          clearFlags();
          moveForward = true;
        }
        digitalWrite(stp_base,HIGH); //Trigger one step forward
        digitalWrite(stp_elbow,HIGH); //Trigger one step forward
        delay(1);
        digitalWrite(stp_base,LOW); //Pull step pin low so it can be triggered again
        digitalWrite(stp_elbow,LOW); //Pull step pin low so it can be triggered again
        delay(1);
      }
      else if(treatValue(analogRead(joyPin1)) < 52){
        velocity(52 - treatValue(analogRead(joyPin1))); // get velocity
        if(!moveBackward){
          digitalWrite(dir_base, HIGH); //Pull direction pin low to move "backward"
          digitalWrite(dir_elbow, LOW); //Pull direction pin low to move "backward"
          digitalWrite(EN_base, LOW);
          digitalWrite(EN_elbow, LOW);
          clearFlags();
          moveBackward = false;
        }
        digitalWrite(stp_base,HIGH); //Trigger one step forward
        digitalWrite(stp_elbow,HIGH); //Trigger one step forward
        delay(1);
        digitalWrite(stp_base,LOW); //Pull step pin low so it can be triggered again
        digitalWrite(stp_elbow,LOW); //Pull step pin low so it can be triggered again
        delay(1);
      }
      else if(treatValue(analogRead(joyPin2)) > 52){
        velocity(abs(52 - treatValue(analogRead(joyPin2)))); // get velocity
        if(!moveUp){
          digitalWrite(dir_elbow, LOW); //Pull direction pin low to move "backward"
          digitalWrite(dir_base, LOW); //Pull direction pin low to move "backward"
          digitalWrite(EN_elbow, LOW);
          digitalWrite(EN_base, LOW);
          clearFlags();
          moveUp = true;
        }
        digitalWrite(stp_elbow,HIGH); //Trigger one step forward
        digitalWrite(stp_base,HIGH); //Trigger one step forward
        delay(1);
        digitalWrite(stp_elbow,LOW); //Pull step pin low so it can be triggered again
        digitalWrite(stp_base,LOW); //Pull step pin low so it can be triggered again
        delay(1);
      }
      else if(treatValue(analogRead(joyPin2)) < 52){
        velocity(52 - treatValue(analogRead(joyPin2))); // get velocity
        if(!moveDown){
          digitalWrite(dir_elbow, HIGH); //Pull direction pin low to move "backward"
          digitalWrite(dir_base, HIGH); //Pull direction pin low to move "backward"
          digitalWrite(EN_elbow, LOW);
          digitalWrite(EN_base, LOW);
          clearFlags();
          moveDown = false;
        }
        digitalWrite(stp_elbow,HIGH); //Trigger one step forward
        digitalWrite(stp_base,HIGH); //Trigger one step forward
        delay(1);
        digitalWrite(stp_elbow,LOW); //Pull step pin low so it can be triggered again
        digitalWrite(stp_base,LOW); //Pull step pin low so it can be triggered again
        delay(1);
      }
      else if(treatValue(analogRead(joyPin2)) == 52 && treatValue(analogRead(joyPin1)) == 52){
        resetBEDPins();
        clearFlags();
      }
      break;
  }
}

void clearFlags(){
  moveForward = false;
  moveBackward = false;
  moveUp = false;
  moveDown = false;
}

void movePen(){
  resetBEDPins();
  long pressTime = millis();
  if((pressTime - previousPressTime) > 2000){
    if(!setPenInPosition){
      setPenInPosition = true;
      for (pos = 0; pos <= 180; pos++) { // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
        gripperServo.write(pos);              // tell servo to go to position in variable 'pos'
      }
      for (pos = 165; pos >= 155; pos--) { // goes from 180 degrees to 90 degrees
        // in steps of 1 degree
        wristServo.write(pos);              // tell servo to go to position in variable 'pos'
      }
    }
    else{
      setPenInPosition = false;
      wristServo.write(165);
      //gripperServo.write(0);
      /*for (pos = 156; pos <= 165; pos ++) { // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
        wristServo.write(pos);              // tell servo to go to position in variable 'pos'
      }
      /*for (pos = 180; pos >= 0; pos--) { // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
        gripperSeo.write(pos); rv             // tell servo to go to position in variable 'pos'
      }*/
    }
    Serial.println(setPenInPosition);
    previousPressTime = millis();
  }
}

// set BED pins to default state
void resetBEDPins()
{
  digitalWrite(stp_elbow, LOW);
  digitalWrite(dir_elbow, LOW);
  digitalWrite(stp_base, LOW);
  digitalWrite(dir_base, LOW);
  digitalWrite(MS1_base, LOW);
  digitalWrite(MS2_base, LOW);
  digitalWrite(MS3_base, LOW);
  digitalWrite(MS1_elbow, LOW);
  digitalWrite(MS2_elbow, LOW);
  digitalWrite(MS3_elbow, LOW);
  digitalWrite(EN_elbow, HIGH);
  digitalWrite(EN_base, HIGH);
}
// set velocity of stepper motors by adjusting the state of MS pins
void velocity(uint8_t x){
  digitalWrite(MS1_base, resolution[x-1][0]);
  digitalWrite(MS2_base, resolution[x-1][1]);
  digitalWrite(MS3_base, resolution[x-1][2]);
  digitalWrite(MS1_elbow, resolution[x-1][0]);
  digitalWrite(MS2_elbow, resolution[x-1][1]);
  digitalWrite(MS3_elbow, resolution[x-1][2]);
}

