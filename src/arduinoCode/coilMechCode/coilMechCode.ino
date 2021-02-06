/*
  This code is to intented to be given serial messages from an odroid running ROS
  to actuate three motors on the perching drone. Two of the motors operate at the same 
  time to close the perching mechanism while the other motor is used to close the 
  charging coil shell to begin the charging of the drone. 

  Author: Glen Greager 
*/

#include <Encoder.h>

//----------------------------------------------------- Pin Declarations 
#define coil_Motor_Speed_Pin 6
#define coil_Motor_Dir_PinA  7
#define coil_Motor_Dir_PinB  8
#define coil_Motor_Enc_PinA 3
#define coil_Motor_Enc_PinB 4 
#define coil_Motor_Enable 10

//----------------------------------------------------- Global variables  
bool showDebugMess = true; 
char message = "0"; 

// ---------------------------------------------------- Function Declarations 

// Attach the encoder 
Encoder coilEnc(coil_Motor_Enc_PinA, coil_Motor_Enc_PinB); 

void setup() //--------------------------------------- Setup 
{
  // Begin Serial Communication 
  Serial.begin(9600);  // Baud Rate is Going to need some love here. 

  // --------------------------------------------------Pin Modes 
 pinMode(coil_Motor_Speed_Pin, OUTPUT); 
 pinMode(coil_Motor_Dir_PinA,  OUTPUT); 
 pinMode(coil_Motor_Dir_PinB,  OUTPUT);
 pinMode(coil_Motor_Enable, OUTPUT); 

}


void loop() // -------------------------------------- Loop 
{

  
  // Monitor Serial Line for Commands 
  message = getTraffic();

  // Perform Motor actions when commands are recieved
  // TO DO determine the final commands  
  switch ( message )
  {
  case '1': 
    break;
  case '2': 
    break; 
  case '3': 
  // Open Coil 
    actuateCoil(1);
    break;
  case '4': 
  // Close Coil 
    actuateCoil(0); 
    break; 
  default:
    // Do nothing because we have earned it. 
    break;
  }
  
  

}// end loop 



// ----------------------------------------------- Functions 
char getTraffic( ) 
{
  // TO DO: Add some resilience here. 
  if ( Serial.available() ) 
  {
    return Serial.read(); 
  }
  else
  {
    return '0';
  }
    
}

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
  delay(800);


  // Monitor the encoder, when the shaft stalls kill the motor power
  int d_dt = 1000; // the derivative of the motor speed, high number just to get in the loop 
  while ( d_dt > 420 ) 
  {
    // Calculate the derivative of encoder counts 
    int v1 = 10*coilEnc.read();  
    delay(10); 
    int v2 = 10*coilEnc.read();  
    d_dt = abs(v2-v1); 


    //Serial.println(d_dt); 
  }

  // Debug message
  Serial.println("The motor has stalled and exited"); 
 
  // Shut the motor off
  analogWrite(coil_Motor_Speed_Pin, 0); 

  // Disable the motor controller 
  digitalWrite(coil_Motor_Enable, LOW); 

}
