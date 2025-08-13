
#include <BTS7960.h>
#include <ps5.h>
#include <ps5Controller.h>
#include <ps5_int.h>

//#include <Stepper.h>

#include <math.h>

//#include <motor.h>
#include <SteeringMotor.h>


struct steeringAngle{
  float angle;
  int inv;
};


const uint8_t L_EN = 23;
const uint8_t R_EN = 22;
const uint8_t L_PWM = 21;
const uint8_t R_PWM = 19;

BTS7960 motorController(L_EN, R_EN, L_PWM, R_PWM);
#include "AS5600.h"
AS5600 as5600;

//hardware constants
float Steering_Ratio = 2.5;

const float Drive_Ratio = 12.375 / 1;
const float Wheel_Diameter = 2;  //inches


const float Wheel_Circumference = Wheel_Diameter * PI;
const float Inches_Per_Motor_Revolution = Wheel_Circumference / Drive_Ratio;

const float steeringOffset = 148.18;

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
SteeringMotor SteerMotor(200, 27, 26, 25, 33,steeringOffset,2.5,1000,1000);
//Stepper stepperA(200, 27, 26, 25, 33);



//DRIVING VARIABLES
int steeringAngle = 0;
int InvDriveSpeed = 1;

//STEERING VARIABLES
float angle = 0;
float targetAngle = 45;
float deltaAngle = targetAngle - angle;


void ps5Update() {
  LXaxis = ps5.LStickX() + .5;
  LYaxis = ps5.LStickY() + .5;
  RXaxis = ps5.RStickX() + .5;
  RYaxis = ps5.RStickY() + .5;
  R2axis = ps5.R2Value();
  L2axis = ps5.L2Value();

  //angle of each stick
  LStickAngle = atan2(LYaxis, LXaxis) * 180 / PI;
  RStickAngle = atan2(RYaxis, RXaxis) * 180 / PI;

  //distance of each stick from it's center
  LStickMagnitude = sqrt(pow(LYaxis, 2) + pow(LXaxis, 2));
  RStickMagnitude = sqrt(pow(RYaxis, 2) + pow(RXaxis, 2));
};

struct steeringAngle angleOptimize(float goal, float angle,int tolerance, bool InvertableDrive){//input the angle with offset already calculated
  struct steeringAngle output;
output.angle = (goal - angle);

  //fit to -360 to 360 range
  while (output.angle < -180) { output.angle += 360; }
  while (output.angle > 180) { output.angle -= 360; }

  //optimizes the path of wheel orientation
  if (InvertableDrive) {
    //if it's faster, drive the wheel backwards
    if (output.angle > 90) {
      output.angle -= 180;
      output.inv = -1;
    }
     else if (output.angle < -90) {
      output.angle += 180;
      output.inv = -1;

    }
     else {
      output.inv = 1;
    }
    }

    //allowable tolerance to reduce noise
    if (abs(output.angle) < tolerance) {
      output.angle = 0;
    }

    return output;
};
/*float OLDStepperGoTo(float goal, Stepper &stepper, AS5600 &Sensor, bool InvertableDrive) {
  angle = Sensor.readAngle() * AS5600_RAW_TO_DEGREES;  //take angle reading from mag encoder
  angle -= steeringOffset;
  //Serial.println(angle);
  
  };

  //convert degrees to the motors units
  int stepCount = round(deltaAngle * 200 / 360);
  if (stepCount > 50) {
    stepCount = 50;
  }
  if (stepCount < -50) {
    stepCount = -50;
  }
  //stepper.step(stepCount * Steering_Ratio);
  //return deltaAngle;
}*/



//updates variables for controller



void setup() {
  Serial.begin(115200);            //communicating with computer
  Wire.begin(14, 15);              //serial bus
  ps5.begin("4C:B9:9B:A5:54:8E");  //stat connecting controller
  motorController.Enable();
  //until the controller is connected
  //while (ps5.isConnected() == false) {
    Serial.println("ps5 waiting");  //let me know
    delay(100);
  //}
  Serial.println("connected");  //let me know when it is connected


  as5600.begin(4);                                //connect to the mag encoder, and declare the direction pin
  as5600.setDirection(AS5600_COUNTERCLOCK_WISE);  //  default, just be explicit.
  //SteerMotor.stepper1.setCurrentPosition(angle*)
  //SteerMotor.stepper1.moveTo(1000);

  //stepperA.setSpeed(100);  //set the speed the stepper will travel in
}

struct steeringAngle steerCalcs;

void loop() {
  //while (!ps5.isConnected()) {
    //StepperGoTo(steeringAngle, stepperA, as5600, true);  //move the stepper the calculated distance
  //}
  angle = as5600.readAngle() * AS5600_RAW_TO_DEGREES;
  ps5Update();

  if (ps5.isConnected()&&(LStickMagnitude > LStickDeadband)) {
    LStickMagnitude *= 255 / 160;
    steeringAngle = -LStickAngle;
    Serial.println(steeringAngle);

    //driveMotor.set(LStickMagnitude*InvDriveSpeed);
    //Serial.println(InvDriveSpeed);
    
  } else {
    steeringAngle = 45;
    motorController.Stop();
    //driveMotor.set(0);
  }

  struct steeringAngle result =  angleOptimize(steeringAngle,angle,5,false);

  //Serial.println(result.angle);
  SteerMotor.ApplySteeringCorrection(result.angle,  angle);
  if (result.inv == 1) {
      motorController.TurnLeft(LStickMagnitude);
    } else {
      motorController.TurnRight(LStickMagnitude);
    }
Serial.println(angle);
  SteerMotor.stepper1.run();
  //Serial.println(steeringAngle-angle);
      //Serial.println(angle-result.angle);

  //Serial.println(SteerMotor.stepper1.targetPosition());
  //Serial.println(result.angle);
  //Serial.println(result.angle);
  //Serial.println(angle);
  //Serial.println(LStickMagnitude);

  //Serial.println(InvDriveSpeed);

}


//  -- END OF FILE -