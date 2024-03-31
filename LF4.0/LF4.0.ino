
#include <QTRSensors.h> //Make sure to install the library

/*************************************************************************
* Sensor Array object initialisation 
*************************************************************************/
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

/*************************************************************************
* PID control system variables 
*************************************************************************/
float Kp = 0.28; //related to the proportional control term; 
              //change the value by trial-and-error (ex: 0.07).
              // Sweet spot: 0.06
float Ki = 0.008; //related to the integral control term; 
              //change the value by trial-and-error (ex: 0.0008).
float Kd = 0.8; //related to the derivative control term; 
              //change the value by trial-and-error (ex: 0.6).
int P;
int I;
int D;

/*************************************************************************
* Global variables
*************************************************************************/
int lastError = 0;
boolean onoff = false;

/*************************************************************************
* Motor speed variables (choose between 0 - no speed, and 255 - maximum speed)
*************************************************************************/
const uint8_t maxspeeda = 255;
const uint8_t maxspeedb = 255;
const uint8_t basespeeda = 240;
const uint8_t basespeedb = 240;

/*************************************************************************
* DRV8835 GPIO pins declaration
*************************************************************************/
// int mode = 8;
int aphase = 6;
int aenbl = 9;
int bphase = 3;
int benbl = 5;

/*************************************************************************
* Buttons pins declaration
*************************************************************************/
int buttoncalibrate = A6; //or pin A3
int buttonstart = A7;
uint16_t position;
char turn_type;

void rotate_right();
void rotate_left();
void test_emmiter();
void forward_brake(int posa, int posb);

int arr[SensorCount];
int thresh = 500;
void toBinaryArray() {
  for (int i = 0; i < SensorCount; i++) {
    arr[i] = sensorValues[i] > thresh;
  }
}

void setup() {
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){8, A0, A1, A2, A3, A4, A5, 7}, SensorCount);
  Serial.begin(9600);
  // qtr.setEmitterPin(7);//LEDON PIN

  // pinMode(mode, OUTPUT);
  pinMode(aphase, OUTPUT);
  pinMode(aenbl, OUTPUT);
  pinMode(bphase, OUTPUT);
  pinMode(benbl, OUTPUT);

  pinMode(buttoncalibrate, INPUT);
  pinMode(buttonstart, INPUT); 
  // digitalWrite(mode, HIGH); //one of the two control interfaces 
                            //(simplified drive/brake operation)
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("Calibrating...");

  // calibration();

  boolean Ok = false;
  while (Ok == false) { // the main function won't start until the robot is calibrated
    if(analogRead(buttoncalibrate) > 700) {
      Serial.println("Started Calibrating");
      calibration(); //calibrate the robot for 10 seconds
      Ok = true;
    }
  }
  // test_emmiter();
  forward_brake(0, 0); //stop the motors
  Serial.println("Calibrated");

  while(analogRead(buttonstart) < 600) {}

  char mode;
}


void calibration() {
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 170; i++)
  {
    qtr.calibrate();
    if (i < 20 ||  i > 50 && i < 80 || i  > 110 && i < 140) {
      rotate_left();
    } else {
      rotate_right();
    }
  }
  digitalWrite(LED_BUILTIN, LOW);
}

int rotation_speed = 110;
void rotate_right() {
  analogWrite(aphase, rotation_speed + 40);
  analogWrite(bphase, 0);
  analogWrite(aenbl, 0);
  analogWrite(benbl, rotation_speed);
}

void rotate_left() {
  analogWrite(aphase, 0);
  analogWrite(bphase, rotation_speed);
  analogWrite(aenbl, rotation_speed + 40);
  analogWrite(benbl, 0);
}

// Code for turning
void turn(char dir)
{
    switch(dir)
    {
    case 'L':
        // Turn left 
        
        forward_brake(-250, 250);
        delay(50);
        while (position > 3600) {
          position = qtr.readLineBlack(sensorValues);
          forward_brake(-250, 250);
        }
        break;
    case 'R':
        // Turn right.
        forward_brake(250, -250);
        delay(50);
        while (position < 3400) {
          position = qtr.readLineBlack(sensorValues);
          forward_brake(250, -250);
        }
        break;
    case 'B':
        // Turn around.
        while (position == 0) {
          position = qtr.readLineBlack(sensorValues);
          forward_brake(250, -250);
        } 
        while (position == 7000) {
          position = qtr.readLineBlack(sensorValues);
          forward_brake(-250, 250);
        }
        break;
    case 'S':
        // Don't do anything!
        break;
    }
}

char select_turn(int found_left, int found_straight, int found_right){
    if(found_right)
        return 'L';
    else if(found_straight)
        return 'S';
    else if(found_left)
        return 'R';
    else
        return 'B';
}

void blinkk (char dir) {
    forward_brake(0, 0);
    delay(1000);
    switch(dir)
    {
    case 'L':
        for (int i = 0; i < 2; i++) {
          digitalWrite(LED_BUILTIN, HIGH);
          delay(200);
          digitalWrite(LED_BUILTIN,  LOW);
          delay(200);
        }
        break;
    case 'R':
        // Turn right.
        for (int i = 0; i < 3; i++) {
          digitalWrite(LED_BUILTIN, HIGH);
          delay(200);
          digitalWrite(LED_BUILTIN,  LOW);
          delay(200);
        }
        break;
    case 'B':
        // Turn around.
        for (int i = 0; i < 4; i++) {
          digitalWrite(LED_BUILTIN, HIGH);
          delay(200);
          digitalWrite(LED_BUILTIN,  LOW);
          delay(200);
        }
        break;
    case 'S':
        // Don't do anything!
        break;
    }
}


void loop() {

  // if(analogRead(buttonstart) > 600) {
  //   onoff =! onoff;
  //   if(onoff = true) {
  //     delay(1000);//a delay when the robot starts
  //   }
  //   else {
  //     delay(50);
  //   }
  // }
  // if (onoff == true) {
  //   PID_control();
  // }
  // else {
  //   forward_brake(0,0); //stop the motors
  // }

  PID_control();
  // delay(500);

  int found_left=0;
  int found_straight=0;
  int found_right=0;

  found_left = (arr[0] || arr[1]) && (arr[3] || arr[4]);
  found_right = (arr[6] || arr[7]) && (arr[3] || arr[4]);

  if (found_left || found_right) {
    forward_brake(250, 250);
    delay(50);
    found_straight = arr[3] || arr[4];
    forward_brake(0, 0);
  }

  if(found_straight || found_left || found_right) {
    turn_type = select_turn(found_left, found_straight, found_right);
    Serial.print(found_left);
    Serial.print(found_straight);
    Serial.println(found_right);

    Serial.println(turn_type);
    blinkk(turn_type);
    turn(turn_type);
  }
  
  if (position == 0) {
    delay(350);
    while (position < 3400) {
      position = qtr.readLineBlack(sensorValues);
      forward_brake(250, -250);
    }
  } else if (position == 7000) {
      delay(350);
      while (position > 3600) {
        position = qtr.readLineBlack(sensorValues);
        forward_brake(-250, 250);
      }
  }

  // print the sensor values as numbers from 0 to 2500, where 0 means maximum
  // reflectance and 2500 means minimum reflectance
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('_');
    Serial.print(arr[i]);
    Serial.print('\t');
  }
  Serial.println();

  toBinaryArray();

  

  // if(arr[0] && arr[1] && arr[2] && arr[3] && arr[4] && arr[5] && arr[6] && arr[7]) {
  //   forward_brake(0, 0);
  //   pinMode(LED_BUILTIN, HIGH);
  //   Serial.println("The End");
  //   while(1) {}
  // }




}


void forward_brake(int posa, int posb) {
  //set the appropriate values for aphase and bphase so that the robot goes straight
  if (posa > 0) {
  digitalWrite(aphase, LOW);
  analogWrite(aenbl, posa);
  } else {
  digitalWrite(aenbl, LOW);
  analogWrite(aphase, -posa);
  }

  if (posb > 0) {
  digitalWrite(bphase, LOW);
  analogWrite(benbl, posb);
  } else {
  digitalWrite(benbl, LOW);
  analogWrite(bphase, -posb);
  }
}



void PID_control() {
  position = qtr.readLineBlack(sensorValues); //read the current position
  int error = 3500 - position; //3500 is the ideal position (the centre)
  Serial.println(position);

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P*Kp + I*Ki + D*Kd; //calculate the correction
                                       //needed to be applied to the speed
  
  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb - motorspeed;
  
  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda;
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = maxspeedb;
  }
  if (motorspeeda < 0) {
    motorspeeda = 0;
  }
  if (motorspeedb < 0) {
    motorspeedb = 0;
  } 
  forward_brake(motorspeeda, motorspeedb);
}