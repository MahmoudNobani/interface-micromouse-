#include <PID_v1.h>
#define IR_pin A1
//A ---> Right Motor
//B----> Left  Motor
#define motorA_pin_en 6
#define motorB_pin_en 5

#define motorA_encoder_pin 2
#define motorB_encoder_pin 3

#define in1_pin 12
#define in2_pin 11
#define in3_pin 10
#define in4_pin 9

#define Rtrig_pin 4
#define Recho_pin 7
#define Ltrig_pin 13
#define Lecho_pin 8

#define RIR_pin A2
#define LIR_pin A0

int distanceL;
int distanceR;
volatile int encoderACounter = 1;
volatile int encoderBCounter = 1;


double OutputB = 70;
double OutputA = 72;


unsigned long prevTime, currentTime;
int interval =60;

void setup()
{


  pinMode(motorA_pin_en, OUTPUT);
  pinMode(motorB_pin_en, OUTPUT);

  pinMode(in1_pin, OUTPUT);
  pinMode(in2_pin, OUTPUT);
  pinMode(in3_pin, OUTPUT);
  pinMode(in4_pin, OUTPUT);

  pinMode(motorA_encoder_pin, INPUT);//Read from Encoder PINS __|¯¯|__|¯¯|__|¯¯|__|¯¯|
  pinMode(motorB_encoder_pin, INPUT);

  //Ultrasonics sensors
  pinMode(Rtrig_pin, OUTPUT);
  pinMode(Ltrig_pin, OUTPUT);
  pinMode(Recho_pin, INPUT);
  pinMode(Lecho_pin, INPUT);

  ////////// TO MAKE SURE MOTORS ARE OFF INITIALY////////////
  digitalWrite(in1_pin, LOW);
  digitalWrite(in2_pin, LOW);
  digitalWrite(in3_pin, LOW);
  digitalWrite(in4_pin, LOW);
  ///////////////////////////////////////////////////////////

  prevTime = millis();

  attachInterrupt(digitalPinToInterrupt(motorA_encoder_pin), motorARevCount, RISING);
  attachInterrupt(digitalPinToInterrupt(motorB_encoder_pin), motorBRevCount, RISING);
  Serial.begin(9600);

}

void loop()
{

  if (millis() - prevTime >= interval) {

    prevTime = millis();
    // IR > 512 no wall(free),,,, IR <512 wall(STOP)
    
    if (analogRead(IR_pin) > 512 && analogRead(LIR_pin) < 512 && analogRead(RIR_pin) < 512){ // if there's a wall on left and right move forward
      MoveForward();
    } else if (analogRead(LIR_pin) > 512){ // There's a no wall on the left 
      NightyDegreesLeft();
      delay(40);
      MoveForwardSlowly();
      }
    else if (analogRead(LIR_pin) < 512 && analogRead(IR_pin) < 512 && analogRead(RIR_pin) > 512)
      {
        NightyDegreesRight();
       delay(40);
        MoveForwardSlowly();}

    else if (analogRead(RIR_pin) < 512) {
      NightyDegreesLeft();
      NightyDegreesLeft();
    }else
    MoveForwardSlowly();
    delay(50);

  } //RIGHT OFF FRONT ON LEFT  ON 



}





void motorARevCount() {
  encoderACounter++;

}


void motorBRevCount() {
  encoderBCounter++;
}




void ErrorCorrection() {
  //Reading from right Ultra
  digitalWrite(Rtrig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(Rtrig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Rtrig_pin, LOW);

  double durationR = pulseIn(Recho_pin, HIGH);
  distanceR = (durationR * .0343) / 2;
  //Reading from LEFT Ultra
  digitalWrite(Ltrig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(Ltrig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Ltrig_pin, LOW);

  double durationL = pulseIn(Lecho_pin, HIGH);
  distanceL = (durationL * .0343) / 2;
  //MotorA      MotorB
  int error = distanceR - distanceL;// EX R=9  L=12 9-12 = -3
  if (error > 5) { //DistanceR > DistanceL --> MotorB > MotorA

    OutputA = 60 ; OutputB = 80;

  }
  if (error == 3 || error == 4 ) {
    OutputA = 68 ;
    OutputB = 72;
  }

  if (error < -5) { // DistanceR < DistanceL --> MotorB < MotorA

    OutputA = 80 ; OutputB = 60;

  }
  if (error == -3 || error == -4 ) {
    OutputA = 72 ;
    OutputB = 68;
  }

  if (error == 1 || error == 2 || error == 0) {
    OutputA = 70 ;
    OutputB = 70;
  }
  if (error == -1 || error == -2) {
    OutputA = 70 ;
    OutputB = 70;
  }


}





void MoveForwardSlowly() {

  //Coding the HBridge for Moving Forward
  //MOTORA
  digitalWrite(in1_pin, LOW);
  digitalWrite(in2_pin, HIGH);
  //MOTORB
  digitalWrite(in3_pin, HIGH);
  digitalWrite(in4_pin, LOW);
  //Adjusting the motors speed to meet the best speed to calibrate the motor
  ErrorCorrection();
  //Giving a speed to the motors
  analogWrite(motorA_pin_en, OutputA-10);
  analogWrite(motorB_pin_en, OutputB-10);

}
void MoveForward() {

  //Coding the HBridge for Moving Forward
  //MOTORA
  digitalWrite(in1_pin, LOW);
  digitalWrite(in2_pin, HIGH);
  //MOTORB
  digitalWrite(in3_pin, HIGH);
  digitalWrite(in4_pin, LOW);
  //Adjusting the motors speed to meet the best speed to calibrate the motor
  ErrorCorrection();
  //Giving a speed to the motors
  analogWrite(motorA_pin_en, OutputA);
  analogWrite(motorB_pin_en, OutputB);

}



void moveBackward() {

  //Coding the HBridge for Moving BackWard
  //MOTORA
  digitalWrite(in1_pin, HIGH);
  digitalWrite(in2_pin, LOW);
  //MOTORB
  digitalWrite(in3_pin, LOW);
  digitalWrite(in4_pin, HIGH);
  //Adjusting the motors speed to meet the best speed to calibrate the motor
  ErrorCorrection();
  //Giving a speed to the motors
  analogWrite(motorA_pin_en, OutputA);
  analogWrite(motorB_pin_en, OutputB);

}


void NightyDegreesLeft() {
  //This Function is to move 90 degrees to the left
  //MOTORA
  digitalWrite(in1_pin, LOW);
  digitalWrite(in2_pin, HIGH);
  //MOTORB
  digitalWrite(in3_pin, LOW);
  digitalWrite(in4_pin, HIGH);

  //Move for 90 deg
  analogWrite(motorA_pin_en, OutputA);
  analogWrite(motorB_pin_en, OutputB);
  delay(26);
  analogWrite(motorA_pin_en, 0);
  analogWrite(motorB_pin_en, 0);
}


void NightyDegreesRight() {
  //This Function is to move 90 degrees to the left
  //MOTORA

  //MOTORA
  digitalWrite(in1_pin, HIGH);
  digitalWrite(in2_pin, LOW);
  //MOTORB
  digitalWrite(in3_pin, HIGH);
  digitalWrite(in4_pin, LOW);

  //Move for 90 deg
  analogWrite(motorA_pin_en, OutputA);
  analogWrite(motorB_pin_en, OutputB);
  delay(26);
  analogWrite(motorA_pin_en, 0);
  analogWrite(motorB_pin_en, 0);

}
