//Quan Nguyen 
//3RRS PM

//libraries
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <TouchScreen.h>
#include <iostream>
#include <cmath>


// Touchscreen Pins (4-wire resistive)
#define XPIN 25 // X+
#define YPIN 26 // Y+
#define XNPIN 32 // X-
#define YNPIN 33 // Y-

#define TS_MINX -2500
#define TS_MAXX 650
#define TS_MINY -2950
#define TS_MAXY 800

#define SCREEN_WIDTH 600
#define SCREEN_HEIGHT 800


//stepper motors
const byte DIR3 = 15;
const byte STEP3 = 2;
const byte DIR2 = 4;
const byte STEP2 = 16; 
const byte DIR1 = 17;
const byte STEP1 = 5; 


float normal[3] = {0,0,1};

float targetX = 300; 
float targetY = 450;
float Xerror = 0;
float Yerror = 0;
bool start = false;

double integralX = 0 ;
double integralY = 0;

float KPX = 0.115;
float KPY = 0.115;
double DPX = 0.0285;
double DPY = 0.0285;
double IPX = 0.008;
double IPY = 0.008;
//starts at -25.875 degrees
// Class
AccelStepper stepperA(AccelStepper::DRIVER, STEP1, DIR1);
AccelStepper stepperB(AccelStepper::DRIVER, STEP2, DIR2);
AccelStepper stepperC(AccelStepper::DRIVER, STEP3, DIR3);
MultiStepper steppers;           // Create instance of MultiStepper



TouchScreen ts(XPIN, YPIN, XNPIN, YNPIN,300); // 300 ohms resistance

float calcPos (float degree){
  int out = round((degree+25.875)/0.225);
  return out ;
}
float prevTime = 0;
float prevXerror =0;
float prevYerror =0;

float prevCoord[2] = {4444,4444};



int C = 1;
float L1 = 44.45;
float L2 = 98.2;
float Top = 79.375;
float Bottom = 51.765;
float H = 110; //78 based

float BottomLoc1[3] = {0, -Bottom, 0};          // [X, Y, Z]
float BottomLoc2[3] = {(-Bottom * sqrt(3)) / 2, Bottom / 2, 0};
float BottomLoc3[3] = {(Bottom * sqrt(3)) / 2,  Bottom / 2, 0};

float vectorDistance(float V1[], float V2[], int size) {
    double sum = 0.0;
    for (int i = 0; i < size; i++) {
        sum += pow(V1[i] - V2[i], 2);
    }
    return sqrt(sum);
}

float result[3] = {0,0,0};
float* calcMotorAngle (float A, float B ){
  
  
  float Z2 = (-Top*(-A*sqrt(3)+B))/(sqrt(4*pow(C,2)+pow((-A*sqrt(3)+B),2))) + H;
  float Y2 = (Top*C)/(sqrt(4*pow(C,2)+pow((-A*sqrt(3)+B),2)));
  float X2 = Y2*-sqrt(3);

  float Z3 = (-Top*(A*sqrt(3)+B))/(sqrt(4*pow(C,2)+pow((A*sqrt(3)+B),2))) + H;
  float Y3 = (Top*C)/(sqrt(4*pow(C,2)+pow((A*sqrt(3)+B),2)));
  float X3 = Y3*sqrt(3);

  float Z1 = (Top*B)/(sqrt(C+pow(B,2))) + H;
  float Y1 = -sqrt(pow(Top,2)-pow(H-Z1,2));
  float X1 = 0;

  float vector1[3] = {X1,Y1,Z1};
  float vector2[3] = {X2,Y2,Z2};
  float vector3[3] = {X3,Y3,Z3};
  float D1 = vectorDistance(vector1,BottomLoc1,3);
  float D2 = vectorDistance(vector2,BottomLoc2,3);
  float D3 = vectorDistance(vector3,BottomLoc3,3);
  

  float angle1 = 90 - (acos(((pow(L2,2)-pow(L1,2)-pow(D1,2))/(-2*L1*D1))) + acos(Z1/D1))*(180/PI);
  float angle2 = 90 - (acos(((pow(L2,2)-pow(L1,2)-pow(D2,2))/(-2*L1*D2))) + acos(Z2/D2))*(180/PI);
  float angle3 = 90 - (acos(((pow(L2,2)-pow(L1,2)-pow(D3,2))/(-2*L1*D3))) + acos(Z3/D3))*(180/PI);

  result[0] = calcPos(angle1);
  result[1] = calcPos(angle2);
  result[2] = calcPos(angle3);

  return result;

}




//stepper motor variables
long pos[3] = {0, 0, 0};                            // An array to store the target positions for each stepper motor
int ENA = 0;                             //enable pin for the drivers

void setup() {
  Serial.begin(115200);

  //Set iniial maximum speed value for the steppers (steps/sec)
  stepperA.setMaxSpeed(200);
  stepperB.setMaxSpeed(200);
  stepperC.setMaxSpeed(200);

  // Adding the steppers to the steppersControl instance for multi stepper control
  steppers.addStepper(stepperA);
  steppers.addStepper(stepperB);
  steppers.addStepper(stepperC);

  calcMotorAngle(0, 0);

 for (int i = 0; i < 3; i++) {
    pos[i] = (long)(result[i]);  // Convert to steps
  }
  //Enable pin
  delay(1000);             //small delay to allow the user to reset the platform



  //Movemement
  steppers.moveTo(pos);  // Calculates the required speed for all motors
  steppers.runSpeedToPosition();  // blocks until all steppers reach their target position
}


  
void loop() {
   TSPoint p = ts.getPoint();

   float Xspeed = 0;
   float Yspeed = 0;
   float distanceTravel = 0;

    int x = map(p.x, TS_MINX, TS_MAXX, 0, SCREEN_WIDTH);
    int y = map(p.y, TS_MINY, TS_MAXY, 0, SCREEN_HEIGHT);
    float coord[2] = {x,y};
   if (prevCoord[0] != 4444){

     distanceTravel = vectorDistance(coord,prevCoord,2);

   }
  double currentTime = millis();
    //Serial.println(prevTime);
    //Serial.println(currentTime);

  double timeChange = max(0.001d, (currentTime - prevTime)/1000.0d);;
  
  
  // Print raw values
  if((p.y != 1023 && x != -108 && y != 803) && start  && distanceTravel/timeChange < 1000){

    prevCoord[0] = x;
    prevCoord[1] = y;
    
    prevTime = currentTime;



    Serial.print("X: "); Serial.print(x);
    Serial.print("  Y: "); Serial.print(y);
    Serial.print("  Pressure: "); Serial.print(p.z);
    Serial.print(" Time:"); Serial.println(millis());
    Xerror = (-targetX + x)/1000;
    Yerror = (-targetY + y)/1000;
  
    double Xchange = 0;
    double Ychange = 0;
    if (prevTime == 0) {
    timeChange = 0.001f; // Default 1ms for first run
    // Skip derivative calculation first time
    Xchange = 0;
    Ychange = 0;
    } else {
    Xchange = (Xerror - prevXerror)/timeChange;
    Ychange = (Yerror - prevYerror)/timeChange;
    }
    
    //Serial.println(prevXerror);
    //Serial.println(prevYerror);
    //Serial.println(Xerror);
    //Serial.println(Yerror);

    
  
  

    prevXerror = Xerror;
    prevYerror = Yerror;

    integralX += Xerror * timeChange;
    integralY += Yerror * timeChange;

    Serial.println(integralX);
    Serial.println(integralY);

     


    double SlopeX = constrain((KPX * Xerror) + (DPX * Xchange) + (IPX*integralX) , -0.3, 0.3);
    double SlopeY = constrain((KPY * Yerror) + (DPY * Ychange) + (IPY*integralY)  , -0.3, 0.3);
    

  calcMotorAngle(SlopeX, SlopeY);

  for (int i = 0; i < 3; i++) {
    pos[i] = (long)(result[i]);  // Convert to steps

  }
  
  //Serial.println(pos[0]);
  //Serial.println(pos[1]);
  //Serial.println(pos[2]);
  
  
  steppers.moveTo(pos);  // Calculates the required speed for all motors
  steppers.runSpeedToPosition(); 


    
  }else if (timeChange > 2 ){
    integralX = 0;
    integralY = 0;
    //Serial.println("Integral Reset");
    calcMotorAngle(0, 0);

  for (int i = 0; i < 3; i++) {
    pos[i] = (long)(result[i]);  // Convert to steps

  }
   steppers.moveTo(pos);  // Calculates the required speed for all motors
  steppers.runSpeedToPosition();
  }
  start = true;



 
  
  delayMicroseconds(100);
  

}
