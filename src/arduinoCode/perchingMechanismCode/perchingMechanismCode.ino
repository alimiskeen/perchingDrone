// This is where I define the pins for the motors
#include <SoftwareSerial.h>
#include <Encoder.h>

// Speed pins
#define Sideone_speed 6
#define Sidetwo_speed 5

// Direction pins
#define SideoneA 12
#define SideoneB 11
#define SidetwoA 10
#define SidetwoB 9

// Enable pin
#define enablePin 4

//Encoder pins
// Motor 1
#define Motor_oneA 3
#define Motor_oneB 8

// Motor 2
#define Motor_twoA 2
#define Motor_twoB 7

// Enocder commands
Encoder coilEnc1(Motor_oneA, Motor_oneB);
Encoder coilEnc2(Motor_twoA, Motor_twoB);

//Encoder pins

char message = '0';

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);

  // Motor 1
  pinMode(SideoneA, OUTPUT);
  pinMode(SideoneB, OUTPUT);
  pinMode(Sideone_speed, OUTPUT);

  // Motor 2
  pinMode(SidetwoA, OUTPUT);
  pinMode(SidetwoB, OUTPUT);
  pinMode(Sidetwo_speed, OUTPUT);

  // Enable pin
  pinMode(enablePin, OUTPUT);

} // end of Setup

void loop()
{
  

  actuateCoil(1); 
  delay(3000); 
  actuateCoil(0); 
  delay(3000); 

  // // Monitor Serial Line for Commands
  // message = getTraffic();

  // // Perform Motor actions when commands are recieved
  // if (message == '6')
  // {
  //   actuateCoil(1);
  // }
  // else if (message == '7')
  // {
  //   actuateCoil(0);
  // }

} // end of loop

char getTraffic()
{
  // TO DO: Add some resilience here.
  if (Serial.available())
  {
    return Serial.read();
  }
  else
  {
    return '0';
  }
}

// Beginning of Actuate func
void actuateCoil(int Direction)
{
  // Enable the motor controllers
  digitalWrite(enablePin, HIGH);

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
  digitalWrite(SideoneA, dirA);
  digitalWrite(SideoneB, dirB);
  analogWrite(Sideone_speed, 200);
  digitalWrite(SidetwoA, dirB);
  digitalWrite(SidetwoB, dirA);
  analogWrite(Sidetwo_speed, 200);

  // Delay to give the motor a chance to overcome friction and get out of stall
  delay(300);

  // Monitor the encoder, when the shaft stalls kill the motor power
  int d_dt1 = 10000; // the derivative of the motor speed
  int d_dt2 = 10000; // the derivative of the motor speed


  while ( (d_dt1 > 30) && (d_dt2 > 30) )
  {
    // Calculate the derivative of encoder counts
    //VA1 is side 1 motor initial read
    //VA2 is side 2 motor initial read

    int vA1 = -10 * coilEnc1.read();
    int vA2 =  10 * coilEnc2.read();
    delay(50);

    // Calculate the derivative of the motors to detect stall 
    d_dt1 = abs(vA1+ (10*coilEnc1.read()) ) / 50;
    d_dt2 = abs(vA2- (10*coilEnc2.read()) ) / 50; 
    // Serial.print(d_dt1);
    // Serial.print("  "); 
    // Serial.print(d_dt2);
    // Serial.print("\t"); 

    // Proportional controller error calculation
    int error1 = vA1 - vA2;

    // Serial.print(error1); 
    // Serial.print("\t");
    // Serial.print(vA1); 
    // Serial.print("\t");
    // Serial.print(vA2);
    // Serial.println(" "); 
    
    // Adjust the speed of motor two based on the error and 
    analogWrite(Sidetwo_speed, 200 - 0.07*error1); 
    
  } // end of while loop

  Serial.println("The motor has stalled and exited"); 

  // Shut off the motors 
  analogWrite(Sideone_speed, 0);
  analogWrite(Sidetwo_speed, 0);

  // disable the motor controllers
  digitalWrite(enablePin, HIGH);  

} // end of function
