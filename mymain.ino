#include<MsTimer2.h>
#include "DualVNH5019MotorShield.h"

#define control_period 0.02  // 20ms

//-------------- tilt sensor variables -----------------//
int period = 0;
int reverse = 0;
double com_roll=0, com_pitch=0;
double roll=0, pitch=0;

#define TILT_DEBUG
//-------------- title sensor variables ----------------//
//----    global variables for IR sensor  start  -------//

#define IRCONSTANT 0.0048828125
#define IR_DEBUG

int IRright = A3;
int IRleft  = A2;  //analog pin number
float distIrRight = -1.f;
float distIrLeft = -1.f;
float A_low_R1 = 0, A_low_R2=0, P_R=0;

 const double LP1_A1 = 0.8816;//0.9391;
 const double LP1_B0 = 0.0592;//0.0305;
 const double LP1_B1 = 0.0592;//0.0305;
 const double HP1_A1 = 0.8816;//0.9391;
const double HP1_B0 = 0.9408;//0.9695;
const double HP1_B1 = -0.9408;//0.9695;

//------- global variables for IR sensor  end -------//
/////    global variables for switch  start ////////
#define MAXSPEED 255
#define MINSPEED -255

#define MAXHEIGHT 100
#define MINHEIGHT 52

#define MAXANGLE 50
#define MINANGLE -10

#define MAXINTEGRAL 25

#define ERROR_WINDOW 50
#define BUTTONDELAY 20
//#define DEBUG_ON

int switchPin = 0;  //swtich circut input connected to analog pin 5
long buttonLastChecked = 0;  //variable to limit the button getting checked every cycle

/////    global variables for switch  end   ////////
//-------------- PID variables ----------------------//
double angleSetpoint=-100.0, angleInput, angleOutput, aIntegral=0, aDiff=0;
double heightSetpoint=-100.0, heightInput, heightOutput, LheightInput, RheightInput, hIntegral=0, hDiff=0;
double rollSetpoint=-100.0, rollInput, rollOutput, rIntegral=0, rDiff=0;
double aError, aLastErr=0, aKp=2, aKi=5, aKd=1;
double hError, hLastErr=0, hKp=2, hKi=5, hKd=1;
double rError, rLastErr=0, rKp=2, rKi=3, rKd=1;
//-------------- PID variables ---------------------//

//--------------- motor driver variables ---------------//
unsigned char _PWM5;
unsigned char _INA3, _INB3;
DualVNH5019MotorShield md;
//--------------- motor driver variables ---------------//

void readIrDst(float *right, float *left){
float voltsRight;
float voltsLeft ;


 //digitalWrite(14+1,HIGH);
 delay(10);
 voltsRight = analogRead(IRright);//*IRCONSTANT;
 delay(10);
 voltsRight = analogRead(IRright);//*IRCONSTANT;
 //digitalWrite(14+2,HIGH);
 delay(10);
 voltsLeft = analogRead(IRleft);//*IRCONSTANT;
 delay(10);
 voltsLeft = analogRead(IRleft);//*IRCONSTANT;
 
 *right = 10650.08 * pow(voltsRight,-0.935) - 10;
 *left = 10650.08 * pow(voltsLeft,-0.935) - 10;
 
  A_low_R1 = LP1_A1 * A_low_R2 + LP1_B0 * (*right) + LP1_B1*P_R;
  P_R = *right;
  *right = A_low_R1;
  A_low_R2 = A_low_R1;     

  #ifdef IR_DEBUG
  Serial.print("IR sensor A3: ");
  Serial.print(*right);
  Serial.print("  IR sensor A2: ");
  Serial.println(*left);
  #endif
  
  delay(100);
}


void readTilt(double* roll, double* pitch){
  loop_MPU6050();
  *roll = com_roll;
  *pitch = com_pitch;
  
  #ifdef TILT_DEBUG
  Serial.print("Roll: ");
  Serial.print(*roll);
  Serial.print("  Pitch: ");
  Serial.println(*pitch);
  #endif
  
  delay(100);
}


void TimerISR()
{
  static boolean output = HIGH;
  period = 1;
}


void initMotor3(){
  _PWM5 = 5;
  _INA3 = 3;
  _INB3 = 11;
  pinMode(_PWM5, OUTPUT);
  pinMode(_INA3, OUTPUT);
  pinMode(_INB3, OUTPUT);
  
}

void setM3Speed(int speed)
{
  int reverse = 0;
  
  if (speed < 0)
  {
    speed = -speed;  // make speed a positive quantity
    reverse = 1;  // preserve the direction
  }
  if (speed > 255)  // Max 
    speed = 255;
  analogWrite(_PWM5,speed); // default to using analogWrite
  if (speed == 0)
  {
    digitalWrite(_INA3,LOW);   // Make the motor coast no
    digitalWrite(_INB3,LOW);   // matter which direction it is spinning.
  }
  else if (reverse)
  {
    digitalWrite(_INA3,LOW);
    digitalWrite(_INB3,HIGH);
  }
  else
  {
    digitalWrite(_INA3,HIGH);
    digitalWrite(_INB3,LOW);
  }
}


int buttonPushed(int pinNum) {
  
    int val = 0;         // variable to store the read value
    digitalWrite((14+pinNum), HIGH); // enable the 20k internal pullup
    delay(10);  //avoid noise value?
    val = analogRead(pinNum);   // read the input pin
    
    #ifdef DEBUG_ON
      Serial.println(val);
      //analogWrite(ledPin, val/4); // analog input 0-1023 while analogWrite 0-255
    #endif
    // we don't use the upper position because that is the same as the
    // all-open switch value when the internal 20K ohm pullup is enabled.
    // if( val >= 923 and val <= 1023 )
    //  Serial.println("switch 0 pressed/triggered");
    if( val >= 830 - ERROR_WINDOW and val <= 830 + ERROR_WINDOW ) {  // 830
      #ifdef DEBUG_ON
      Serial.println("switch 1 pressed/triggered");
      #endif
      return 1;
    }
    else if ( val >= (610-ERROR_WINDOW) and val <= (610+ERROR_WINDOW) ) { // 610
      #ifdef DEBUG_ON
      Serial.println("switch 2 pressed/triggered");
      #endif
      return 2;
    }
    else if ( val >= (420-ERROR_WINDOW) and val <= (420+ERROR_WINDOW) ) { // 420
      #ifdef DEBUG_ON
      Serial.println("switch 3 pressed/triggered");
      #endif
      return 3;
    }
    else if ( val >= (220-ERROR_WINDOW) and val <= (220+ERROR_WINDOW) ) { // 220
      #ifdef DEBUG_ON
      Serial.println("switch 4 pressed/triggered");
      #endif
      return 4;
    }
    else if( val >= 0 and val <= (20+ERROR_WINDOW) )  {
      #ifdef DEBUG_ON
      Serial.println("switch 5 pressed/triggered");    
      #endif
      return 5;
    }
    else
      return 0;  // no button found to have been pushed
}

//////////////***PID***///////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\//////////////\\\\\\\\/////////////
void anglePID(double angleSetpoint)
{
  if(angleSetpoint>MAXANGLE)
  angleSetpoint=MAXANGLE;
  else if(angleSetpoint<MINANGLE)
  angleSetpoint=MINANGLE;
  
  angleInput = pitch;
  //aLastErr=aError;
  aError=angleSetpoint-angleInput;
  if(abs(angleSetpoint) <0.1){
    if(abs(angleInput) < 0.5)
      return;
  }
  if(abs(aError) < 2)
    return;
  //aDiff=(aLastErr-aError)/0.01;
  angleOutput=aKp*aError+aKi*aIntegral;//+aKd*aDiff;
  if(angleOutput>MAXSPEED)
    angleOutput=MAXSPEED;
  else if(angleOutput<MINSPEED)
    angleOutput=MINSPEED;
  aIntegral=+aError; 
 if(aIntegral>MAXINTEGRAL)
  aIntegral=MAXINTEGRAL; 
  Serial.println(angleOutput);
// putting drive value in motor
   for(int i=0; i<=10; i++)
   {
       setM3Speed(angleOutput);
       delay(2);   
   } 
}

void heightPID(double heightSetpoint)
{
  LheightInput =distIrLeft; 
  RheightInput =distIrRight;
  if (heightSetpoint>MAXHEIGHT)
    heightSetpoint=MAXHEIGHT; 
  else if(heightSetpoint<MINHEIGHT)
    heightSetpoint=MINHEIGHT;
  heightInput=(LheightInput+RheightInput)/2;
  if (max(LheightInput,RheightInput)>MAXHEIGHT){
    if(LheightInput > MAXHEIGHT)
      LheightInput = MAXHEIGHT;
    if(RheightInput > MAXHEIGHT)
      RheightInput = MAXHEIGHT;
  }
  else if(min(LheightInput,RheightInput)<MINHEIGHT){
    if(LheightInput < MINHEIGHT)
      LheightInput = MINHEIGHT;
    if(RheightInput < MINHEIGHT)
      RheightInput = MINHEIGHT;
  }
  //hLastErr= hError;
  hError=heightSetpoint-heightInput;
  //hDiff=(hLastErr-hError)/0.01;
  heightOutput=hKp*hError+hKi*hIntegral;//+hKd*hDiff;
  hIntegral=hIntegral+hError; 
  if(abs(hIntegral)>MAXINTEGRAL)
    {
      if (hIntegral>0)
      hIntegral=MAXINTEGRAL;
      if(hIntegral<0)
      hIntegral=-MAXINTEGRAL;
    }
  Serial.println(hError);
  Serial.println(hIntegral);
  Serial.println(heightOutput);
  if(abs(hError) <= 0.5)
    return;
  // putting drive value in motor
  for(int i=0; i<=10; i++)
  {
    md.setM1Speed(heightOutput);
    delay(2);   
  }  
  for(int i=0; i<=10; i++)
  {
    md.setM2Speed(heightOutput);
    delay(2);   
  }  
}

void rollPID()
{
 double rollInput=roll;
 double rollSetpoint=0;
 if(abs(rollInput)< 0.3)
   return;
 //rLastErr=rError;
 rError=rollSetpoint-rollInput;
 //rDiff=(rLastErr-rError)/0.01;
 rollOutput=rKp*rError+rKi*rIntegral;//+rKd*rDiff;
 rIntegral=rIntegral+rError; 
 if(abs(rIntegral)>MAXINTEGRAL)
  {
    if (rIntegral>0)
    rIntegral=MAXINTEGRAL;
    if(rIntegral<0)
    rIntegral=-MAXINTEGRAL;
  }
 LheightInput =distIrLeft; 
 RheightInput =distIrRight;
 Serial.println(rollOutput);
 if (max(LheightInput,RheightInput)<MAXHEIGHT && min(LheightInput,RheightInput)>MINHEIGHT )
 {
   for(int i=0; i<=10; i++)
   {
     //left
     md.setM1Speed(-rollOutput);
     delay(2);   
   }  
   for(int i=0; i<=10; i++)
   {
     //right
     md.setM2Speed(rollOutput);
     delay(2);   
   }
 }
 else if (LheightInput>MAXHEIGHT || LheightInput <MINHEIGHT)
 {
   for(int i=0; i<=10; i++)
   {
     //right
     md.setM2Speed(rollOutput);
     delay(2);   
   }
 }
 else if (RheightInput>MAXHEIGHT || RheightInput <MINHEIGHT)
 {
   for(int i=0; i<=10; i++)
   {
     //left
     md.setM1Speed(-rollOutput);
     delay(2);   
   }  
 }
} 
//////////////////////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\////////////////


///////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\/////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\/////////
void setup(){
  
  //initialize the variables we're linked to
  //Input = analogRead(0);
  //Setpoint = 50;

  ////////////////////////////////////////////////
  Serial.begin(115200);
  
  MsTimer2::set(control_period*1000, TimerISR);
  MsTimer2::start();
  
  initMotor3();
  md.init();
 
  setup_MPU6050(); // tilt sensor setup
  
//  pinMode(A1,OUTPUT);
//  digitalWrite(14+1,LOW);
//  pinMode(A4,OUTPUT);
//  digitalWrite(14+4,LOW);
//  pinMode(A5,OUTPUT);
//  digitalWrite(14+5,LOW);
//  
  //pinMode(IRleft,INPUT);
  //pinMode(IRright, INPUT);
  //digitalWrite(14+2,HIGH);  
}

void loop(){


   readIrDst(&distIrRight, &distIrLeft);
   readTilt(&roll,&pitch);
   
   if(angleSetpoint == -100.0){
     angleSetpoint = pitch;
   }
   if(rollSetpoint == -100.0){
     rollSetpoint = roll;
   }
   if(heightSetpoint == -100.0){
     heightSetpoint = (distIrRight + distIrLeft)/2.0;
   }
   
   
// switch code start //  
  if( buttonLastChecked == 0 ) // see if this is the first time checking the buttons
    buttonLastChecked = millis()+BUTTONDELAY;  // force a check this cycle
  if( millis() - buttonLastChecked > BUTTONDELAY ) { // make sure a reasonable delay passed
    if( int buttNum = buttonPushed(switchPin) ) {
      Serial.print("Button "); Serial.print(buttNum); Serial.println(" was pushed.");
     //switch (buttNum)
     //{
     //  case 1:
     if(buttNum==1){
      //rollPID();
       angleSetpoint=0;
       anglePID(angleSetpoint);
       //heightSetpoint=70;
       //heightPID(heightSetpoint);
       //break;
       }       
       //case 2:
       else if (buttNum==2){
       angleSetpoint=angleSetpoint+2;
       Serial.print("===================angle Setpoint=============");
       Serial.println(angleSetpoint);
       anglePID(angleSetpoint);
       //break;
       }
       //case 3:
       else if (buttNum==3){
       angleSetpoint=angleSetpoint-2;
       Serial.print("===============angle Setpoint=================");
       Serial.println(angleSetpoint);
       anglePID(angleSetpoint);
       //break;
       }
       //case 4:
       else if (buttNum==4){
       heightSetpoint=heightSetpoint+2;
       Serial.print("================height Setpoint===============");
       Serial.println(heightSetpoint);
       heightPID(heightSetpoint);
       //break;
       }
       //case 5:
       else if (buttNum==5){
       heightSetpoint=heightSetpoint-2;
       Serial.print("=================height Setpoint==============");
       Serial.println(heightSetpoint);
       heightPID(heightSetpoint);
       //break;
     }
    }
    buttonLastChecked = millis(); // reset the lastChecked value
  }  
// switch code end //
////////////////**** TESTING CODE****//////////////////////
//angleSetpoint=0;
//anglePID(angleSetpoint);

//heightSetpoint=60;
//heightPID(heightSetpoint);

//rollPID();
//////////////////////////////////////////////////////////`````  
}
