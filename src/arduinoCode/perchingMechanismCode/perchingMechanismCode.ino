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

char message[4] = {'0','0','0','0'};
int  coilCommand = 0;

//---------------------------------------------------------------------- setup()
void setup()
{
  // Establish Serial Connection
  Serial.begin(9600);
  // Ping to odroid what this device is 
  Serial.print("gripper"); 

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

// ---------------------------------------------------------------------- loop()
void loop()
{


//  actuateCoil(1);
//  delay(3000);
//  actuateCoil(0);
//  delay(3000);

  // // Monitor Serial Line for Commands
  if (Serial.available() > 3) {
    getTraffic();

    Serial.print(message);
  }

  // // Perform Motor actions when commands are recieved
  switch (message[3]) 
  {
    case '4':
      actuateCoil(0); break; 
    case '3':
      actuateCoil(1); break; 
    case '2':
      // Send message to other arduino
      break; 
    case '1':
      // Send message to other arduino
      break; 
    default: 
      Serial.println("Recieved a Junk command, comm must be bad"); 
  }

  // Reset the message variable
   message[0] = '0';
   message[1] = '0';
   message[2] = '0';
   message[3] = '0';

} // end of loop

// ---------------------------------------------------------------- getTraffic()
char getTraffic()
{

  // Arduino is expecting the code with "C10" as the first three characters
  // This should add some resilience in the communication
  if (Serial.available() > 3) {
    message[0] = Serial.read();
    Serial.println(message[0]);
    if ( message[0] == 'C') {
      message[1] = Serial.read();
      Serial.println(message[1]);
      if ( message[1] == '1') {
        message[2] = Serial.read();
        Serial.println(message[2]);
        if (message[2] == '0'){
          message[3] = Serial.read();
          //Serial.println(message[3]);
          //coilCommand = int(Serial.read());

        }
      }
    }

  }
}

// -------------------------------------------------------------- actuateCoil()
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
  analogWrite(Sideone_speed, 255);
  digitalWrite(SidetwoA, dirB);
  digitalWrite(SidetwoB, dirA);
  analogWrite(Sidetwo_speed, 255);

  // Delay to give the motor a chance to overcome friction and get out of stall
  delay(300);

  // Monitor the encoder, when the shaft stalls kill the motor power
  int d_dt1 = 10000; // the derivative of the motor speed
  int d_dt2 = 10000; // the derivative of the motor speed


  while ( (d_dt1 > 20)  )
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

    // Proportional controller error calculation
    int error1 = vA1 - vA2;


    // Adjust the speed of motor two based on the error and
    analogWrite(Sidetwo_speed, 200 - 0.07*error1);

  } // end of while loop

  Serial.println("The motor has stalled and exited");

  // Shut off the motors
  analogWrite(Sideone_speed, 0);
  analogWrite(Sidetwo_speed, 0);

  // disable the motor controllers
  digitalWrite(enablePin, HIGH);

} //------------------------------------------------------- end of actuateCoil()
