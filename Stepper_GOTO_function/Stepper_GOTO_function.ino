#include <ps5.h>
#include <ps5Controller.h>
#include <ps5_int.h>
#include <Stepper.h>
#include <math.h>
#include <motor.h>


#include "AS5600.h"
 AS5600 as5600; 


    int driveMotor_pwm = 19;
    int driveMotor_FWD = 5;
    int driveMotor_BWD = 18;
  Motor_L928N driveMotor= Motor_L928N(driveMotor_FWD,driveMotor_BWD,driveMotor_pwm,true);

float LXaxis;
float LYaxis;
float RXaxis;
float RYaxis;
float R2axis;
float L2axis;
float LStickAngle = 0;
float RStickAngle = 0;
float LStickMagnitude = 0;
float RStickMagnitude = 0;

int steeringAngle=0;


Stepper stepperA(200, 27, 26, 25, 33);
float angle = 0;
float targetAngle = 45;
float deltaAngle = targetAngle-angle;
int InvDriveSpeed = 1;
float LStickDeadband = 10;
float RStickDeadband = 10;



float StepperGoTo(float goal, Stepper &stepper, AS5600 &Sensor,bool InvertableDrive ){
  angle = Sensor.readAngle()*AS5600_RAW_TO_DEGREES;
  deltaAngle = (goal-angle);
  while (deltaAngle<-180){deltaAngle+=360;}
  while (deltaAngle>180){deltaAngle-=360;}
  if(InvertableDrive){
    
    if(deltaAngle>90){
      deltaAngle -=180;
      InvDriveSpeed=-1;
    }
    else if(deltaAngle<-90){
      deltaAngle +=180;
      InvDriveSpeed=-1;
     
    }
    else {InvDriveSpeed=1;}

    if(abs(deltaAngle)<2){
      deltaAngle = 0;
    }
    
  };
  int stepCount = round(deltaAngle*200/360);
  stepper.step(stepCount);
  return deltaAngle;
}


void ps5Update(){
  LXaxis=ps5.LStickX()+.5;
  LYaxis=ps5.LStickY()+.5;
  RXaxis=ps5.RStickX()+.5;
  RYaxis=ps5.RStickY()+.5;
  R2axis=ps5.R2Value();
  L2axis=ps5.L2Value();

  LStickAngle = atan2(LYaxis,LXaxis)*180/PI;
  RStickAngle = atan2(RYaxis,RXaxis)*180/PI;

   LStickMagnitude = sqrt(pow(LYaxis,2)+pow(LXaxis,2));
   RStickMagnitude = sqrt(pow(RYaxis,2)+pow(RXaxis,2));

  
};


void setup()
{
  Serial.begin(115200);
  Wire.begin(14, 15);
  ps5.begin("4C:B9:9B:A5:54:8E");
  while(ps5.isConnected()==false){
    Serial.println("ps5 waiting");
    delay(100);}
    Serial.println("connected");


  as5600.begin(4);  //  set direction pin.
  as5600.setDirection(AS5600_COUNTERCLOCK_WISE);  //  default, just be explicit.
  

  stepperA.setSpeed(240);
  
}
void loop()
{
  
  ps5Update();
  //Serial.println(LStickMagnitude);
  if(LStickMagnitude>LStickDeadband){
    steeringAngle = -LStickAngle;
    driveMotor.set(LStickMagnitude*InvDriveSpeed);
    }
  else{
    steeringAngle = steeringAngle;
   driveMotor.set(0);
  }
  StepperGoTo(steeringAngle,stepperA,as5600,true);
  Serial.println(InvDriveSpeed);

  delay(10);
  
}


//  -- END OF FILE --
