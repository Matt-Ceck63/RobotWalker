#include <Adafruit_PWMServoDriver.h>

#define SERVOMIN 80
#define SERVOMAX 555
#define SERVONUM 16

//Define pins
#define leftArmPin 0
#define leftHipPin 8
#define leftThighPin 9
#define leftKneePin 10
#define leftFootPin 11

#define rightArmPin 4
#define rightHipPin 12
#define rightThighPin 13
#define rightKneePin 14
#define rightFootPin 15

//degrees
#define leftArmStr 180
#define leftHipStr 86
#define leftThighStr 100 //Decrease = pull leg back
#define leftKneeStr 60 //Decrease = inverted knee, increase = natural bend 
#define leftFootStr 55 //60 = flat, 0 = tip-toe

#define rightArmStr 0
#define rightHipStr -1 //60 replace servo
#define rightThighStr 102 //Increase = pull leg back
#define rightKneeStr 85 //Increase = inverted knee, decrease = natural bend
#define rightFootStr 95 //90 = flat, 180 = tip-toe

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

typedef struct Servo
{
  int pin;
  float currPos;
  float targetPos;
  float targetDifference;
  float speedMultiplier;
  int newCommand; //0 not going to move, 1 going to move
  float stepSize;
  float newPos;
  bool targetReached;
  bool detached;

  // Constructors
  Servo(){}
  Servo(int _pin, int _currPos)
  {
    pin = _pin;
    currPos = map(_currPos, 0, 180, SERVOMIN, SERVOMAX);
    targetPos = map(_currPos, 0, 180, SERVOMIN, SERVOMAX);
    targetDifference = 0;
    speedMultiplier = 0;
    newCommand = 0;
    stepSize = 0;
    newPos = 0;
    targetReached = false;
    detached = false;

    if(_currPos < 0)
    {
      targetReached = true;  
      detached = true;
      currPos = -1;
      detachServo();
    }
    else
    {
      //Initialize pos
      pwm.setPWM(pin, 0, currPos);
    }       
  }

  void setTargetWithSpeed(int _targetPos, float _speedMultiplier) //target Pos in degrees, Use this command to move this servo
  {
    speedMultiplier = _speedMultiplier;
    targetPos = map(_targetPos, 0, 180, SERVOMIN, SERVOMAX);
    targetReached = false;
    targetDifference = targetPos - currPos;
    newPos = 0;
    if(_targetPos > 0)
      newCommand = 1;
    if(targetDifference == 0)
      newCommand = 0;
  }

  void detachServo()
  {
    detached = true;
    pwm.setPWM(pin, 0, 0);
  }

};

Servo *servos[SERVONUM];
int defaultAngles[SERVONUM] = {-1,-1,-1,-1,-1,-1,-1,-1, leftHipStr, leftThighStr, leftKneeStr, leftFootStr, rightHipStr, rightThighStr, rightKneeStr, rightFootStr};
int squatAngles[SERVONUM] = {-1,-1,-1,-1,-1,-1,-1,-1, leftHipStr, leftThighStr+45, leftKneeStr+50, leftFootStr+10, rightHipStr, rightThighStr-45, rightKneeStr-50, rightFootStr-10};
float unSquatSpeeds[SERVONUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 30, 15, 25, 0, 30, 15, 25};
int shuffleLeftLeg[SERVONUM] = {-1,-1,-1,-1,-1,-1,-1,-1, leftHipStr, leftThighStr+20, leftKneeStr+30, leftFootStr+10, rightHipStr, rightThighStr, rightKneeStr, rightFootStr};
float shuffleLeftLegSpeeds[SERVONUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 15, 15, 15, 0, 15, 15, 15};
int rightForward[SERVONUM] = {-1,-1,-1,-1,-1,-1,-1,-1, leftHipStr, leftThighStr+20, leftKneeStr+30, leftFootStr+10, rightHipStr, rightThighStr-20, rightKneeStr-30, rightFootStr-10};
float rightForwardSpeeds[SERVONUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 15, 15, 15, 0, 15, 15, 15};
int shuffleRightLeg[SERVONUM] = {-1,-1,-1,-1,-1,-1,-1,-1, leftHipStr, leftThighStr, leftKneeStr, leftFootStr, rightHipStr, rightThighStr-25, rightKneeStr-35, rightFootStr-10};
float shuffleRightLegSpeeds[SERVONUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 15, 15, 15, 0, 15, 15, 15};
int leftForward[SERVONUM] = {-1,-1,-1,-1,-1,-1,-1,-1, leftHipStr, leftThighStr+20, leftKneeStr+30, leftFootStr+10, rightHipStr, rightThighStr-20, rightKneeStr-30, rightFootStr-10};
float leftForwardSpeeds[SERVONUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 15, 15, 15, 0, 15, 15, 15};

void setup()
{
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(1000);

  //Initializing servos
  for(int i = 0; i < SERVONUM; i++)
  {
    servos[i] = new Servo(i, defaultAngles[i]); //'new' returns a pointer to the object
  }

  delay(1000);
}

void loop()
{

}

void squat()
{
  moveTo(defaultAngles, 15);
  delay(3000);
  moveToWithSpeeds(defaultAngles, unSquatSpeeds);
  delay(3000);
  moveTo(squatAngles, 15);
  delay(3000);
}

void walk()
{
  moveToWithSpeeds(shuffleLeftLeg, shuffleLeftLegSpeeds);
  delay(1500);
  moveToWithSpeeds(rightForward, rightForwardSpeeds);
  delay(1500);
  moveToWithSpeeds(defaultAngles, unSquatSpeeds);
  delay(1500);
  moveToWithSpeeds(shuffleRightLeg, shuffleRightLegSpeeds);
  delay(1500);
  moveToWithSpeeds(leftForward, leftForwardSpeeds);
  delay(1500);
  moveToWithSpeeds(defaultAngles, unSquatSpeeds);
  delay(1500);
}

void moveToWithSpeeds(int angles[], float speed[])
{
  for(int i = 0; i < SERVONUM; i++)
  {
    servos[i]-> setTargetWithSpeed(angles[i], speed[i]);
  }
  moveJoints();
}

void moveTo(int angles[], float speed)
{
  for(int i = 0; i < SERVONUM; i++)
  {
    servos[i]-> setTargetWithSpeed(angles[i], speed);
  }
  moveJoints();
}

void moveJoints()
{
  //Find and store servos than need to move
  int servoCount = 0;
  for(int i = 0; i < SERVONUM; i++)
  {
    if((servos[i]->newCommand) == 1) servoCount++;
  }

  Servo *servosMove[servoCount];
  servoCount = 0;
  for(int i = 0; i < SERVONUM; i++)
  {
    if((servos[i]->newCommand) == 1) {
      servosMove[servoCount] = servos[i];
      servoCount++;
    }
  }

  //Find stepsize with time for each, stepsize can be positive or negative
  for(int i = 0; i < servoCount; i++)
  {
    servosMove[i]->stepSize = (servosMove[i]->targetDifference)/(abs((servosMove[i]->targetDifference)));//((servosMove[i]->targetDifference)/smallestTargetDifference)/((servosMove[i]->time)/smallestTime);
  }
  
  bool stopWritingServos = false;
  while(!stopWritingServos)
  {
    for(int i = 0; i < servoCount; i++)
    {
      moveSingleServo(servosMove[i], float(servoCount)); 
    }

    stopWritingServos = true;
    for(int i = 0; i < servoCount; i++)
    {
      if(!(servosMove[i]->targetReached)) stopWritingServos = false;
    }
  }
}

//Every servo movement is independent of each other
void moveSingleServo(Servo *i, float servoCount)
{
  if(i->targetReached) return;
  
  float calc = sin((PI/(i->targetDifference))*((i->newPos)));
  //Serial.print("calc: "); Serial.println(calc);

  //clamp maximum of the sin curve
  if((i->stepSize) < 0)
    if (calc < -0.5) calc = -0.5;
  
  if((i->stepSize) > 0)
    if(calc > 0.5) calc = 0.5;

  //step size can be positive or negative
  float step = ((i->speedMultiplier)/(servoCount)) * (i->stepSize + 1 * calc);
  //Serial.println(MAP_TIME(i->time));
  //Serial.print("Step: "); Serial.println(step);
  i->currPos += step;
  i->newPos += abs(step);
  
  if((i->stepSize) < 0)
    if (i->currPos <= (i->targetPos)) i->targetReached = true;
    
  if((i->stepSize) > 0)
    if (i->currPos >= (i->targetPos)) i->targetReached = true;
  
  pwm.setPWM(i->pin, 0, i->currPos);
}

//  //Find smallest anglePwm difference
//  float smallestTargetDifference = SERVOMAX;
//  for(int i = 0; i < servoCount; i++)
//  {
//    if(abs(servosMove[i]->targetDifference) < smallestTargetDifference) 
//       smallestTargetDifference = abs(servosMove[i]->targetDifference);
//  }
//   
//  //Find smallest time difference
//  long smallestTime = 60000; //1 minute
//  for(int i = 0; i < servoCount; i++)
//  {
//    if((servosMove[i]->time) < smallestTime) 
//      smallestTime = (servosMove[i]->time);
//  }
//  
