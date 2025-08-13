#include <AccelStepper.h>
#include <MultiStepper.h>

#ifndef __SteeringMotor_H__
#define __SteeringMotor_H__

//#include <Stepper.h>
#include <math.h>



class SteeringMotor{

  public:
    float  SPR;//steps per revolution
    int PinA;
    int PinB;
    int PinC;
    int PinD;
    float Offset;
    float Ratio;
    int MaxSpeed;
    int MaxAccel;
    AccelStepper stepper1;

    SteeringMotor(int spr,int pinA,int pinB,int pinC, int pinD, float offset, float ratio,int  maxSpeed,int maxAccel){
      
      SPR = spr;
      PinA = pinA;
      PinB = pinB;
      PinC = pinC;
      PinD = pinD;
      Ratio = ratio;
      MaxSpeed=maxSpeed;
      Offset = offset;
      MaxAccel=maxAccel;

    
      stepper1=AccelStepper(AccelStepper::FULL4WIRE, PinA, PinB,PinC, PinD);
      stepper1.setMaxSpeed(maxSpeed);
      stepper1.setAcceleration(maxAccel);
    }
    
   void ApplySteeringCorrection(float deltaAngle, float encoderAngle) {
      
    deltaAngle= Ratio*deltaAngle;//steps needed to rotate
    stepper1.move(deltaAngle*200/360-stepper1.targetPosition());
    //Serial.println(deltaAngle);
    //stepper1.move((deltaAngle*200/360)-stepper1.targetPosition());
  //}

}



/*
    void set(int spd){
      this->SPD=spd;
    
      if(spd>=0){
        digitalWrite(FWD_PIN,0);
        digitalWrite(BKD_PIN,1);
      }
      if(spd<0){
        digitalWrite(FWD_PIN,1);
        digitalWrite(BKD_PIN,0);
      }
      //analogWrite(this->SPD_PIN,abs(spd));
      
      analogWrite(SPD_PIN,abs(spd));
      
      }

    int getSpeed(){
        return SPD;
    }*/
};



#endif