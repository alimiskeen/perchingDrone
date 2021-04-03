/*
  This code is to intented to be given serial messages from an odroid running ROS
  to actuate three motors on the perching drone. Two of the motors operate at the same 
  time to close the perching mechanism while the other motor is used to close the 
  charging coil shell to begin the charging of the drone. 

  Author: Glen Greager 
*/

#include <Encoder.h>

//----------------------------------------------------- Pin Declarations 
#define coil_Motor_Speed_Pin  6
#define coil_Motor_Dir_PinA   7
#define coil_Motor_Dir_PinB   8
#define coil_Motor_Enc_PinA   3
#define coil_Motor_Enc_PinB   4 
#define coil_Motor_Enable     10
#define commandPin A7

//----------------------------------------------------- Global variables  
bool showDebugMess = true; 
bool coilIsOpen = false;


// ---------------------------------------------------- Function Declarations 

// Attach the encoder 
Encoder coilEnc(coil_Motor_Enc_PinA, coil_Motor_Enc_PinB); 

char message[4] = {'0','0','0','0'};
int  coilCommand = 0;

void setup() //--------------------------------------- Setup 
{
  // Begin Serial Communication 
  Serial.begin(9600);  // Baud Rate is Going to need some love here. 

  // --------------------------------------------------Pin Modes 
 pinMode(coil_Motor_Speed_Pin, OUTPUT); 
 pinMode(coil_Motor_Dir_PinA,  OUTPUT); 
 pinMode(coil_Motor_Dir_PinB,  OUTPUT);
 pinMode(coil_Motor_Enable, OUTPUT); 
 pinMode(commandPin, INPUT); 

  // Make sure the coil starts in the open position 
  //actuateCoil(0);
  //coilIsOpen = true;  
 
}


void loop() // -------------------------------------- Loop 
{

  // Monitor the command pin 
  int coilPinState = analogRead(commandPin); 
  delay(200); 

  Serial.print("CoilPinState: "); 
  Serial.print(coilPinState);
  Serial.print("\t Core Status:"); 
  Serial.println(coilIsOpen); 



   
 // Logic to decide what to do 
 if (coilPinState < 200 & !coilIsOpen) //
 {
   // Coil is being commanded open, so open it 
   actuateCoil(0); 
   coilIsOpen = true;
 }
 else if ( (coilPinState > 950) & coilIsOpen )
 {
   // Coil is being commanded closed, so close it 
   actuateCoil(1); 
   coilIsOpen = false; 
 }


}// end loop 



// ----------------------------------------------- Functions 


void actuateCoil(int Direction) 
{
  // Enable the motor controller 
  digitalWrite(coil_Motor_Enable, HIGH); 

  // Define the direction Variables 
  int dirA = 0; 
  int dirB = 0; 

  // Decide Which Direction we need to go
  if (Direction == 0)
  {
    dirA = 1; 
    dirB = 0; 
  }
  else
  {
    dirA = 0; 
    dirB = 1; 
  }

  // Turn the motors on in the respective direction
  digitalWrite(coil_Motor_Dir_PinA, dirA); 
  digitalWrite(coil_Motor_Dir_PinB, dirB); 
  analogWrite(coil_Motor_Speed_Pin, 255);  

  // Delay to give the motor a chance to overcome friction and get out of stall
  delay(5
  00);


  // Monitor the encoder, when the shaft stalls kill the motor power
  int d_dt = 1000; // the derivative of the motor speed, high number just to get in the loop 
  while ( d_dt > 450 ) 
  {
    // Calculate the derivative of encoder counts 
    int v1 = 10*coilEnc.read();  
    delay(10); 
    int v2 = 10*coilEnc.read();  
    d_dt = abs(v2-v1); 


    Serial.println(d_dt); 
  }

  // Debug message
  Serial.println("The motor has stalled and exited"); 
 
  // Shut the motor off
  analogWrite(coil_Motor_Speed_Pin, 0); 

  // Disable the motor controller 
  digitalWrite(coil_Motor_Enable, LOW); 

}
