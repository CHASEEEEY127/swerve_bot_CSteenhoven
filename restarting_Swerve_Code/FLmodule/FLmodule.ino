
#include <ps5.h>
#include <ps5Controller.h>
#include <ps5_int.h>
#include <Stepper.h>
#include <math.h>

#include <BTS7960.h>
#include "AS5600.h"

int inv =1;
struct swerveTrajectory{float angle;int inv; float speed;};
struct swerveTrajectory optimizeAngle(int goal,int current,bool invertable,int maxDegrees, float speed){
    int deltaAngle = (goal-current);
    speed*=(cos(deltaAngle*M_PI/180));
    if(abs(deltaAngle)<2){
      //Serial.println(deltaAngle);
      return {0,inv,speed};
    }
    goal=goal%360;
    current=current%360;
    if(invertable){
      
      while((abs(deltaAngle) >=180)){
          inv*=-1;
          deltaAngle-= 180*deltaAngle/abs(deltaAngle);

          }
      if(abs(deltaAngle)>90){
                  inv*=-1;

          deltaAngle-= 180*deltaAngle/abs(deltaAngle);
      }
      

      if(deltaAngle ==90&&inv==-1){//this covers the 270 degree edgecase, where inverting the speed of the motor happens unnecassarily, when it is just as fast rotationally, to not invert speed.
          deltaAngle*=-1;
          inv*=-1;
      }
    }
    if(abs(deltaAngle)>maxDegrees){
      deltaAngle = maxDegrees*deltaAngle/abs(deltaAngle);
    }
    return {deltaAngle,inv,speed};
};




const uint8_t L_EN = 23;
const uint8_t R_EN = 22;
const uint8_t L_PWM = 21;
const uint8_t R_PWM = 19;

BTS7960 motorController(L_EN, R_EN, L_PWM, R_PWM);

 AS5600 absEncoder; 


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
int InvDriveSpeed = 1;

//STEERING VARIABLES
float angle = 0;
float targetAngle = 45;
float deltaAngle = targetAngle-angle;
float angleOffset = -141.75-9.5;
swerveTrajectory Vector={0,0,0};


/*
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
*/
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
  motorController.Enable();


  absEncoder.begin(4); //connect to the mag encoder, and declare the direction pin
  absEncoder.setDirection(AS5600_COUNTERCLOCK_WISE);  //  default, just be explicit.
  
  stepperA.setSpeed(100);//set the speed the stepper will travel in
  for(int i =0; i<10;i++){
    angle = (absEncoder.readAngle()*AS5600_RAW_TO_DEGREES)+angleOffset;
    Vector = optimizeAngle(45,angle,false,360,0);
    stepperA.step((Vector.angle*200/360)*2.5);
    }
    
    
  //until the controller is connected
  while(ps5.isConnected()==false){
    Serial.println("ps5 waiting");//let me know
    delay(100);}
    Serial.println("connected");//let me know when it is connected

}
void loop()
{
  //update all the ps5 variables
  ps5Update();
  angle = (absEncoder.readAngle()*AS5600_RAW_TO_DEGREES)+angleOffset;
  
   if(LStickMagnitude>LStickDeadband){
    steeringAngle = -LStickAngle;
   }
  else{
    steeringAngle = 45;
    motorController.Stop();
    LStickMagnitude = 0;
  }
  //Vector = optimizeAngle(steeringAngle,angle,true,100,100);
  Vector = optimizeAngle(steeringAngle,angle,true,45,LStickMagnitude/1.5);
  //Serial.println(Vector.speed);
  //Serial.println(Vector.inv);
    if (Vector.speed < 0) {
      motorController.TurnLeft(abs(Vector.speed));
    } 
    else {
      motorController.TurnRight(abs(Vector.speed));
    }    
  stepperA.step(round(Vector.angle*200/360/4));
  Serial.println(angle-45);
  //Serial.println(LStickMagnitude);
  //StepperGoTo(steeringAngle,stepperA,as5600,true);//move the stepper the calculated distance

  //Serial.println(InvDriveSpeed);

  //delay(10);
  
}


//  -- END OF FILE --