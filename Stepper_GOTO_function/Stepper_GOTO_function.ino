#include <ps5.h>
#include <ps5Controller.h>
#include <ps5_int.h>
#include <Stepper.h>
#include <math.h>
#include <motor.h>


#include "AS5600.h"
 AS5600 as5600; 

//DRIVE MOTOR DECLARATIONS
    int driveMotor_pwm = 19;
    int driveMotor_FWD = 5;
    int driveMotor_BWD = 18;
  Motor_L928N driveMotor= Motor_L928N(driveMotor_FWD,driveMotor_BWD,driveMotor_pwm,true);


//CONTROLLER CONSTANTS
float LStickDeadband = 10;
float RStickDeadband = 10;

//CONTROLLER VARIABLES
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


//STEPPER
Stepper stepperA(200, 27, 26, 25, 33);



//DRIVING VARIABLES
int steeringAngle=0;
Int InvDriveSpeed = 1;

//STEERING VARIABLES
float angle = 0;
float targetAngle = 45;
float deltaAngle = targetAngle-angle;




float StepperGoTo(float goal, Stepper &stepper, AS5600 &Sensor,bool InvertableDrive ){
  angle = Sensor.readAngle()*AS5600_RAW_TO_DEGREES; //take angle reading from mag encoder

  deltaAngle = (goal-angle);

  //fit to -360 to 360 range
  while (deltaAngle<-180){deltaAngle+=360;}
  while (deltaAngle>180){deltaAngle-=360;}

  //optimizes the path of wheel orientation
  if(InvertableDrive){
    //if it's faster, drive the wheel backwards
    if(deltaAngle>90){
      deltaAngle -=180;
      InvDriveSpeed=-1;
    }
    else if(deltaAngle<-90){
      deltaAngle +=180;
      InvDriveSpeed=-1;
     
    }
    else {InvDriveSpeed=1;}

  //allowable tolerance to reduce noise
    if(abs(deltaAngle)<2){
      deltaAngle = 0;
    }
    
  };

  //convert degrees to the motors units
  int stepCount = round(deltaAngle*200/360);
  stepper.step(stepCount);
  return deltaAngle;
}

//updates variables for controller
void ps5Update(){
  LXaxis=ps5.LStickX()+.5;
  LYaxis=ps5.LStickY()+.5;
  RXaxis=ps5.RStickX()+.5;
  RYaxis=ps5.RStickY()+.5;
  R2axis=ps5.R2Value();
  L2axis=ps5.L2Value();

  //angle of each stick
  LStickAngle = atan2(LYaxis,LXaxis)*180/PI;
  RStickAngle = atan2(RYaxis,RXaxis)*180/PI;

  //distance of each stick from it's center
   LStickMagnitude = sqrt(pow(LYaxis,2)+pow(LXaxis,2));
   RStickMagnitude = sqrt(pow(RYaxis,2)+pow(RXaxis,2));
  
};


void setup()
{
  Serial.begin(115200); //communicating with computer
  Wire.begin(14, 15);//serial bus
  ps5.begin("4C:B9:9B:A5:54:8E");//stat connecting controller

//until the controller is connected
  while(ps5.isConnected()==false){
    Serial.println("ps5 waiting");//let me know
    delay(100);}
    Serial.println("connected");//let me know when it is connected


  as5600.begin(4); //connect to the mag encoder, and declare the direction pin
  as5600.setDirection(AS5600_COUNTERCLOCK_WISE);  //  default, just be explicit.
  

  stepperA.setSpeed(240);//set the speed the stepper will travel in
  
}
void loop()
{
  //update all the ps5 variables
  ps5Update();
 
   if(LStickMagnitude>LStickDeadband){
    steeringAngle = -LStickAngle;
    driveMotor.set(LStickMagnitude*InvDriveSpeed);
    }
  else{
    steeringAngle = steeringAngle;
   driveMotor.set(0);
  }

  StepperGoTo(steeringAngle,stepperA,as5600,true);//move the stepper the calculated distance

  //Serial.println(InvDriveSpeed);

  delay(10);
  
}


//  -- END OF FILE --
