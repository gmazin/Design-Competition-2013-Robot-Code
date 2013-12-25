#include <NewPing.h>
#include "States.h"
#define MOTOR_RIGHT_PWM 5
#define MOTOR_RIGHT_DIR 4
#define MOTOR_LEFT_PWM 6
#define MOTOR_LEFT_DIR 2

#define MOTOR_RIGHT_DUTY 210
#define MOTOR_LEFT_DUTY 195

#define MOTOR_RIGHT_TURN_DUTY 110
#define MOTOR_LEFT_TURN_DUTY 110

#define UNIVERSAL_TIMEOUT 7000
#define TIMEOUT 4000
#define HOME_TIMEOUT 4000

#define LASER_NUM 3
#define RIGHT_LASER_OUTPUT 10
#define LEFT_LASER_OUTPUT 11
#define CAPTURE_LASER_OUTPUT 13
#define HOME_LASER_OUTPUT 12

#define RIGHT_LASER_INPUT 3
#define LEFT_LASER_INPUT 0
#define CAPTURE_LASER_INPUT 2
#define HOME_LASER_INPUT 4

#define BIG_LASER_THRESHOLD 200
#define LASER_THRESHOLD 180
#define CAPTURE_THRESHOLD 100
#define BIG_CAPTURE_THRESHOLD 200
#define LASER_FREQUENCY 15
#define BOOL_FREQUENCY 5
 
#define RIGHT_TRIGGER_PIN  9  // Arduino pin tied to trigger pin on ping sensor.
#define RIGHT_ECHO_PIN     8  // Arduino pin tied to echo pin on ping sensor.

#define LEFT_TRIGGER_PIN 7
#define LEFT_ECHO_PIN 3

#define SONAR_NUM     2 // Number of sensors.
#define FILTER_FREQUENCY 5
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 50 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
#define SONAR_THRESHOLD 22

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN, MAX_DISTANCE)
};

States state = Start;
States last_state = Start;

int right_laser = 0;
int left_laser = 0;
int capture_laser = 0;
int home_laser = 0;
boolean right_detect;
boolean left_detect;
boolean home_detect;
boolean right_sonar_detect;
boolean left_sonar_detect;

boolean capture_flag = false;
boolean seeking_on_flag = false;
boolean seeking_off_flag = false;
boolean seeking_tried = false;

unsigned long end_time;
unsigned long universal_end = UNIVERSAL_TIMEOUT;
unsigned long seeking_start;
unsigned long seeking_end = HOME_TIMEOUT;
unsigned long seeking_timeout = HOME_TIMEOUT;
int catches = 0;
int total_prey = 0;




unsigned int laser_readings[LASER_NUM][LASER_FREQUENCY];
boolean laser_detects[LASER_NUM][BOOL_FREQUENCY];


void setup() {
  Serial.begin(115200);
  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++){ // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
  }
  for(uint8_t i = 0; i < SONAR_NUM; i++){
    cm[i] = MAX_DISTANCE;
  }
   
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  
  pinMode(RIGHT_LASER_OUTPUT, OUTPUT);
  pinMode(LEFT_LASER_OUTPUT, OUTPUT);
  pinMode(RIGHT_LASER_INPUT, INPUT);
  pinMode(LEFT_LASER_INPUT, INPUT);
  pinMode(CAPTURE_LASER_OUTPUT, OUTPUT);
  pinMode(CAPTURE_LASER_INPUT, INPUT);
  pinMode(HOME_LASER_OUTPUT, OUTPUT);
  
  init_capture();
}

void loop() {
 
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      //if(filter_iteration[currentSensor] < FILTER_FREQUENCY){
        cm[currentSensor] = MAX_DISTANCE;                      // Make distance zero in case there's no ping echo for this sensor.
        sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
      //}
    }
  }
  
  //capture_laser = analogRead(CAPTURE_LASER_INPUT);
  //Serial.println(capture_laser);
  // Other code that *DOESN'T* analyze ping results can go here.
  
  if(millis() > universal_end){
    catches = 0;
    backUp();
    delay(1000);
    turnLeft();
    delay(1500);
    changeState(SweepLeft);
  } else if(state == Start){
    charge();
    delay(750);
    changeState(SweepRight);
  }
  else if(state == SweepRight){
    Serial.println("State: Sweep Right");
    sweepRight();
  } else if(state == SweepLeft){
    Serial.println("State: Sweep Left");
    sweepLeft();
  }else if(state == TurnRightBuffer){
    Serial.println("State: Right Buffer");
    bufferRight();
  } else if(state == TurnLeftBuffer){
    Serial.println("State: Left Buffer");
    bufferLeft();
  } else if(state == Capture){
    Serial.println("State: Capture");
    capture();
  } else if(state == WallOnRight){
    Serial.println("State: Wall On Right");
    wallRight();
  } else if(state == WallOnLeft){
    Serial.println("State: Wall On Left");
    wallLeft();
  } else if(state == SeekingHome){
    Serial.println("State: ET Go Home");
    seekingHome();
  } else if(state == FoundFront){
    Serial.println("State: ET Find Front Door");
    approachFront();
  } else if(state == FoundBack) {
    Serial.println("State: ET Find Back Wall");
    approachBack();
  } else if(state == Deposit){
    Serial.println("State: Drop Kids Off");
    dropOff();
  }
 
  
/*
  left_laser = analogRead(LEFT_LASER_INPUT);
  right_laser = analogRead(RIGHT_LASER_INPUT);
  capture_laser = analogRead(CAPTURE_LASER_INPUT);
  home_laser = analogRead(HOME_LASER_INPUT);
  printLaser();
  
  */
  
  //Serial.println(capture_laser);
  
  /*
  Serial.print("Left: ");
  Serial.print(leftLaserDetect());
  Serial.print(" Right: ");
  Serial.println(rightLaserDetect());
  */
  //read_lasers();
  
  /*
  read_lasers();
  //if(!leftLaserDetect()){
  //printLaser();
  Serial.print("Left: ");
  Serial.print(left_detect);
  Serial.print(" Right: ");
  Serial.println(right_detect);
  */
  //}
  
  
  //delay(100);
  
}

void init_capture()
{
  digitalWrite(RIGHT_LASER_OUTPUT, HIGH);
  digitalWrite(LEFT_LASER_OUTPUT, HIGH);
  digitalWrite(HOME_LASER_OUTPUT, LOW);
  digitalWrite(CAPTURE_LASER_OUTPUT, LOW);
}

void init_homeseeking()
{
  digitalWrite(RIGHT_LASER_OUTPUT, LOW);
  digitalWrite(LEFT_LASER_OUTPUT, LOW);
  digitalWrite(HOME_LASER_OUTPUT, HIGH);
  digitalWrite(CAPTURE_LASER_OUTPUT, HIGH);
  seeking_end = millis() + HOME_TIMEOUT;
  //seeking_timeout = millis() + HOME_TIMEOUT;
}

void charge()
{
  analogWrite(MOTOR_RIGHT_PWM, 255);
  analogWrite(MOTOR_LEFT_PWM, 227);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  digitalWrite(MOTOR_LEFT_DIR, LOW);
}

void driveStraight()
{
  analogWrite(MOTOR_RIGHT_PWM, MOTOR_RIGHT_DUTY);
  analogWrite(MOTOR_LEFT_PWM, MOTOR_LEFT_DUTY);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  digitalWrite(MOTOR_LEFT_DIR, LOW);
}

void turnLeft()
{
  analogWrite(MOTOR_RIGHT_PWM, MOTOR_RIGHT_TURN_DUTY);
  analogWrite(MOTOR_LEFT_PWM, 255-MOTOR_LEFT_TURN_DUTY);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  digitalWrite(MOTOR_LEFT_DIR, HIGH);
}

void leanLeft()
{
  analogWrite(MOTOR_RIGHT_PWM, MOTOR_RIGHT_TURN_DUTY);
  analogWrite(MOTOR_LEFT_PWM, MOTOR_LEFT_TURN_DUTY/3);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  digitalWrite(MOTOR_LEFT_DIR, LOW);
}

void turnRight()
{
  analogWrite(MOTOR_RIGHT_PWM, 255-MOTOR_RIGHT_TURN_DUTY);
  analogWrite(MOTOR_LEFT_PWM, MOTOR_LEFT_TURN_DUTY);
  digitalWrite(MOTOR_RIGHT_DIR, HIGH);
  digitalWrite(MOTOR_LEFT_DIR, LOW);
}

void leanRight()
{
  analogWrite(MOTOR_RIGHT_PWM, MOTOR_RIGHT_TURN_DUTY/3);
  analogWrite(MOTOR_LEFT_PWM, MOTOR_LEFT_TURN_DUTY);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  digitalWrite(MOTOR_LEFT_DIR, LOW);
}

void backUp()
{
  analogWrite(MOTOR_RIGHT_PWM, 0);
  analogWrite(MOTOR_LEFT_PWM, 0);
  digitalWrite(MOTOR_RIGHT_DIR, HIGH);
  digitalWrite(MOTOR_LEFT_DIR, HIGH);
}

void freezeMotors()
{
  analogWrite(MOTOR_RIGHT_PWM, 0);
  analogWrite(MOTOR_LEFT_PWM, 0);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  digitalWrite(MOTOR_LEFT_DIR, LOW);
}

void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
  // The following code would be replaced with your code that does something with the ping results.
 
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    Serial.print(i);
    Serial.print("=");
    Serial.print(cm[i]);
    Serial.print("cm ");
  }
  Serial.println();
  
  /*
  sonar_readings[0] = find_median(cm[0], FILTER_FREQUENCY);
  sonar_readings[1] = find_median(cm[1], FILTER_FREQUENCY);
  for(uint8_t i = 0; i < SONAR_NUM; i++){
    Serial.print(i);
    Serial.print("=");
    Serial.print(sonar_readings[i]);
    Serial.print("cm ");
  }
  
  for(uint8_t i = 0; i < SONAR_NUM; i++){
    filter_iteration[i] = 0;
  }
  Serial.println();
  */
}

void printLaser()
{
  Serial.print("0: ");
  Serial.print(left_laser);
  Serial.print(" 1: ");
  Serial.print(right_laser);
  Serial.print(" 2: ");
  Serial.print(capture_laser);
  Serial.print(" 3: ");
  Serial.print(home_laser);
  Serial.println();
}

void sweepRight()
{
  checkSonar();
  if(right_sonar_detect){
    changeState(WallOnRight);
  } else if(left_sonar_detect){
    changeState(WallOnLeft);
  } else {
    turnRight();
    read_lasers();
    if(right_detect && !home_detect){
      changeState(TurnRightBuffer);
    } else if(left_detect && !home_detect){
      changeState(TurnLeftBuffer);
    }
  }
}

void sweepLeft()
{
  checkSonar();
  if(right_sonar_detect){
    changeState(WallOnRight);
  } else if(left_sonar_detect){
    changeState(WallOnLeft);
  } else {
    turnLeft();
    read_lasers();
    if(right_detect && !home_detect){
      changeState(TurnRightBuffer);
    } else if(left_detect && !home_detect){
      changeState(TurnLeftBuffer);
    }
  }
}

void bufferRight()
{
  checkSonar();
  if(right_sonar_detect){
    changeState(WallOnRight);
  } else if(left_sonar_detect){
    changeState(WallOnLeft);
  } else {
    turnRight();
    read_lasers();
    if(!right_detect){
      changeState(Capture);
      end_time = millis()+TIMEOUT;
      Serial.println(end_time);
    }
  }
}

void bufferLeft()
{
  checkSonar();
  if(right_sonar_detect){
    changeState(WallOnRight);
  } else if(left_sonar_detect){
    changeState(WallOnLeft);
  } else {
    turnLeft();
    read_lasers();
    if(!left_detect){
      changeState(Capture);
      end_time = millis()+TIMEOUT;
      Serial.println(end_time);
    }
  }
}

void capture()
{
  checkSonar();
  if(right_sonar_detect){
    digitalWrite(CAPTURE_LASER_OUTPUT, LOW);
    changeState(WallOnRight);
  } else if(left_sonar_detect){
    digitalWrite(CAPTURE_LASER_OUTPUT, LOW);
    changeState(WallOnLeft);
  } else {
    if(millis() > end_time){
      Serial.println(millis());
      digitalWrite(CAPTURE_LASER_OUTPUT, LOW);
      backUp();
      if(last_state == TurnRightBuffer){
        changeState(SweepLeft);
      } else if(last_state == TurnLeftBuffer){
        changeState(SweepRight);
      }
    } else {
      digitalWrite(CAPTURE_LASER_OUTPUT, HIGH);
      capture_laser = analogRead(CAPTURE_LASER_INPUT);
      Serial.println(capture_laser);
      if(capture_laser > BIG_CAPTURE_THRESHOLD){
        if(catches == 0){
          if(!capture_flag){
            capture_flag = true;
          }
          /*
          if(capture_laser < BIG_CAPTURE_THRESHOLD){
            if(!capture_flag){
              capture_flag = true;
            }
          } else {
            if(capture_flag){
              capture_flag = false;
              delay(25);
              digitalWrite(CAPTURE_LASER_OUTPUT, LOW);
              catches++;
              changeState(SweepRight);
            }
          }*/
        } else if(catches == 1){
          digitalWrite(CAPTURE_LASER_OUTPUT, LOW);
          catches++;
          changeState(SeekingHome);
        }
      } else {
        if(catches == 0){
          if(capture_flag){
            capture_flag = false;
            digitalWrite(CAPTURE_LASER_OUTPUT, LOW);
            catches++;
            changeState(SweepRight);
            return;
          }
        }
        driveStraight();
        read_lasers();
        if(right_detect){
          changeState(TurnRightBuffer);
        } else if(left_detect){
          changeState(TurnLeftBuffer);
        }
      }
    }
  }
}

void wallRight()
{
  turnLeft();
  checkSonar();
  if(!right_sonar_detect && !left_sonar_detect){
    changeState(SweepLeft);
  }
}

void wallLeft()
{
  turnRight();
  checkSonar();
  if(!right_sonar_detect && !left_sonar_detect){
    changeState(SweepRight);
  }
}


void seekingHome()
{
  turnRight();
  for(uint8_t i = 0; i < BOOL_FREQUENCY; i++){
    home_laser = analogRead(HOME_LASER_INPUT);
    Serial.println(home_laser);
    laser_detects[2][i] = homeLaserDetect();
  }
  home_detect = find_median(laser_detects[2], BOOL_FREQUENCY);
  Serial.println(home_detect);
  
  if(home_detect){
    changeState(FoundFront);
  } else {
    if(millis() > seeking_end){
      changeState(FoundBack);
    }
  }
  
  /*
  if(home_detect){
    if(!seeking_on_flag){
      seeking_on_flag = true;
      seeking_start = millis();
    } else if(seeking_off_flag){
      seeking_end = millis();
      seeking_on_flag = false;
      seeking_off_flag = false;
      Serial.println(seeking_end - seeking_start);
      if(seeking_end - seeking_start < 1000){
        changeState(FoundFront);
      } 
    }
  } else {
    if(!seeking_on_flag){
      if(millis() > seeking_timeout){
        changeState(FoundBack);
      }
    } else {
      seeking_off_flag = true;
    }
  }*/
}

void approachBack()
{
  catches = 0;
  backUp();
  delay(1000);
  turnLeft();
  delay(1500);
  changeState(SweepRight);
}

void approachFront()
{
  charge();
  checkSonar();
  Serial.print("Left: ");
  Serial.print(cm[0]);
  Serial.print(" Right: ");
  Serial.println(cm[1]);
  if(right_sonar_detect || left_sonar_detect){
    changeState(Deposit);
  }
}

void dropOff()
{
  catches = 0;
  backUp();
  delay(1000);
  turnLeft();
  delay(1000);
  changeState(SweepLeft);
}

void changeState(States s)
{
  if(s == SeekingHome){
    init_homeseeking();
  }
  last_state = state;
  state = s;
  freezeMotors();
  delay(25);
  universal_end = millis() + UNIVERSAL_TIMEOUT;
}

void checkSonar()
{
  left_sonar_detect = cm[0] < SONAR_THRESHOLD;
  right_sonar_detect = cm[1] < SONAR_THRESHOLD;
}
/*
int partition(unsigned int array[], int left, int right)
{
  int high = right-1;
  unsigned int pivot = array[high];
  int swap_index = left;
  
  for(uint8_t i = left; i < high; i++){
    if(array[i] < pivot){
      swap(array, swap_index, i);
      swap_index++;
    }
  }
  swap(array, swap_index, high);
  return swap_index;
}

unsigned int find_median(unsigned int array[], int length)
{
  int left = 0;
  int right = length;
  int k = length/2; //only use arrays with odd number of elements
  int pivot_index;
  
  
  
  while(1){
    pivot_index = partition(array, left, right);
    
    if(pivot_index == k){
      return array[pivot_index];
    } else if(pivot_index < k){
      left = pivot_index + 1;
    } else {
      right = pivot_index;
    }
  }
}*/

int partition(boolean array[], int left, int right)
{
  int high = right-1;
  boolean pivot = true;
  int swap_index = left;
  
  for(uint8_t i = left; i <= high; i++){
    if(array[i] < pivot){
      swap(array, swap_index, i);
      swap_index++;
    }
  }
  return swap_index;
}

boolean find_median(boolean array[], int length)
{
  int left = 0;
  int right = length;
  int k = length/2; //only use arrays with odd number of elements
  int pivot_index;
  
  
  
  
  pivot_index = partition(array, left, right);
  
  if(pivot_index == k){
    return array[pivot_index];
  } else if(pivot_index < k){
    return true;
  } else {
    return false;
  }
  
}

void swap(boolean array[], int i, int j)
{
  boolean temp = array[i];
  array[i] = array[j];
  array[j] = temp;
}
/*
void swap(unsigned int array[], int i, int j)
{
  unsigned int temp = array[i];
  array[i] = array[j];
  array[j] = temp;
}*/

void read_lasers()
{
  /*
  for(uint8_t k = 0; k < BOOL_FREQUENCY; k++){
    for(uint8_t i = 0; i < LASER_FREQUENCY; i++){
      laser_readings[0][i] = analogRead(LEFT_LASER_INPUT);
      laser_readings[1][i] = analogRead(RIGHT_LASER_INPUT);
    }
    left_laser = find_median(laser_readings[0], LASER_FREQUENCY);
    right_laser = find_median(laser_readings[1], LASER_FREQUENCY);
    laser_detects[0][k] = leftLaserDetect();
    laser_detects[1][k] = rightLaserDetect();
  }
  
  left_detect = find_median(laser_detects[0], BOOL_FREQUENCY);
  right_detect = find_median(laser_detects[1], BOOL_FREQUENCY);
  */
  for(uint8_t i = 0; i < BOOL_FREQUENCY; i++){
    left_laser = analogRead(LEFT_LASER_INPUT);
    right_laser = analogRead(RIGHT_LASER_INPUT);
    home_laser = analogRead(HOME_LASER_INPUT);
    /*
    Serial.print("Left: ");
    Serial.print(left_laser);
    Serial.print(" Right: ");
    Serial.print(right_laser);
    Serial.print(" Home: ");
    Serial.print(home_laser);
    Serial.println();*/
    laser_detects[0][i] = leftLaserDetect();
    laser_detects[1][i] = rightLaserDetect();
    laser_detects[2][i] = homeLaserDetect();
  }
  
  left_detect = find_median(laser_detects[0], BOOL_FREQUENCY);
  right_detect = find_median(laser_detects[1], BOOL_FREQUENCY);
  home_detect = find_median(laser_detects[2], BOOL_FREQUENCY);
  //left_laser = analogRead(LEFT_LASER_INPUT);
  //right_laser = analogRead(RIGHT_LASER_INPUT);
  Serial.print("Left: ");
  Serial.print(left_detect);
  Serial.print(" Right: ");
  Serial.print(right_detect);
  Serial.print(" Home: ");
  Serial.print(home_detect);
  Serial.println();
}

boolean leftLaserDetect()
{
  return (left_laser >= BIG_LASER_THRESHOLD && right_laser >= BIG_LASER_THRESHOLD && 2*left_laser >= right_laser) || (left_laser >= LASER_THRESHOLD && left_laser > 2*right_laser) || (left_laser >= BIG_LASER_THRESHOLD && right_laser < BIG_LASER_THRESHOLD);
}

boolean rightLaserDetect()
{
  return (left_laser >= BIG_LASER_THRESHOLD && right_laser >= BIG_LASER_THRESHOLD && 2*right_laser >= left_laser) || (right_laser >= LASER_THRESHOLD && right_laser > 2*left_laser) || (right_laser >= BIG_LASER_THRESHOLD && left_laser < BIG_LASER_THRESHOLD);
}

boolean homeLaserDetect()
{
  return home_laser > 130;
}
