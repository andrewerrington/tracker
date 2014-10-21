// Program to drive two RC servomotors to point an antenna to
// a requested azimuth/elevation position.

// By Andrew Errington ZL3AME
// December 2012

// This software was originally written to work in conjunction
// with 'predict' by KD2BD to track satellites.  'predict' sends
// AZ/EL position requests in a string via a serial port.  The
// string format is "AZnnn.n ELnn.n"

// AZ is 0-359 degrees.  0 is true North
// EL is 0-90 degrees.  0 is horizontal, 90 is vertical.

// We use two servomotors with 180 degree resolution, mounted
// perpendicular to each other.  Although they only have 180
// degrees of movement we can cover 360 degrees of azimuth by
// utilising the 180 - 90 degrees half of the elevation servo's
// range to point at the 'other side'.

// The Futaba S3001 servos operate like this (view from above):

//          90
//          |
// 180 -    *    - 0

// Initialise by moving to AZ/EL 0/0.  This should set the
// azimuth servo to 90 degrees and the elevation servo to
// 0 degrees.

// This should make the beam point horizontally due N.  Use a
// compass and level to verify this.

// Enter main loop
// Get AZ/EL from serial port.  Expect it no more than once per
// second.  Optionally, when data feed stops, move to AZ/EL 0/0
// after a timeout

// If AZ or EL movement is a large distance from previous AZ/EL
// then make smaller intermediate steps.

#include <Servo.h> 

// Azimuth and Elevation servo objects
Servo AZservo;  
Servo ELservo;

// Azimuth and Elevation servo pins
int AZservopin=9;
int ELservopin=10;

// Servo min/max timing values in microseconds
// XXservomin is 0 degrees, XXservomax is 180 degrees
// XXservo_zero_is_min records the physical location of
// 0 degrees, i.e. whether the min value represents 0 degrees
// fixme: store these in Flash and make then configurable via
// the serial interface
int ELservomin = 620;
int ELservomax = 2330;
int ELservo_zero_is_min=true;

int AZservomin = 670;
int AZservomax = 2450;
int AZservo_zero_is_min=false;

int slot_time_ms = 100;  // How often we check for serial data
int reset_timer = 0;     // Timeout timer if there is no data
int reset_limit_ms = 30000;  // Move to 0/0 after this time

int minAZperslot = 3;   // Small servo movement permitted
int minELperslot = 3;
int maxAZperslot = 15;  // Large servo movement permitted
int maxELperslot = 15;

int LEDpin = 13;  // Use the on-board LED

void setup()
{
  pinMode(LEDpin, OUTPUT);
  digitalWrite(LEDpin,LOW);
  Serial.begin(9600);
  AZservo.attach(AZservopin,AZservomin,AZservomax);
  ELservo.attach(ELservopin,ELservomin,ELservomax);
  
  // Drive servos to AZ/EL 0/0, due North, 0 degrees above horizon.
  // This corresponds to 90 degress on the azimuth servo and
  // 0 degrees on the elevation servo
  delay(1000);
  driveAZ(90);  // AZ servo to midpoint
  delay(500);
  driveEL(0);   // EL servo horizontal
}

void loop()
{

  // Variables for requested antenna position  
  static int AZ=0;  // Antenna position is set to 0/0 by setup()
  static int EL=0;
  
  // Variables for calculated servo positions
  static int AZservo_pos=90;  // AL servo at 90 degrees by setup()
  static int ELservo_pos=0;   // EL servo at 0 degrees by setup()

  static int last_AZservo_pos=90;
  static int last_ELservo_pos=0;
  
  int new_servo_pos;
  
  // Listen to serial port for "AZnnn.nn ELnn.nn" messages.
  // We can discard the decimal part.
  
  while (Serial.available() > 0)
  {
    if (Serial.read() != 'A')
    {
      break;
    }
    // We got "A".  Assume it is the beginning of the string.
    digitalWrite(LEDpin,HIGH);
    reset_timer=0;
    // look for the next valid integer in the incoming data
    AZ = (int)Serial.parseFloat();
    // do it again:
    EL = (int)Serial.parseFloat();
 
    Serial.print("Requested AZ:");
    Serial.print(AZ);
    Serial.print("\n");
    Serial.print("Requested EL:");
    Serial.print(EL);
    Serial.print("\n");

    digitalWrite(LEDpin,LOW); 
  }

  // Convert the requested antenna position to servo positions
  translateAZEL(AZ,EL,&AZservo_pos,&ELservo_pos);
  
  // Allow small movements.  Limit large movements.
  if (abs(last_AZservo_pos-AZservo_pos)<minAZperslot)
  {
    last_AZservo_pos = AZservo_pos;
  }
  else
  {
    new_servo_pos = (last_AZservo_pos+AZservo_pos)/2;
    if (abs(new_servo_pos-AZservo_pos)>maxAZperslot)
    {
      if (new_servo_pos > last_AZservo_pos)
      {
        last_AZservo_pos += maxAZperslot;
      }
      else
      {
        last_AZservo_pos -= maxAZperslot;
      }      
    }
    else
    {
      last_AZservo_pos = new_servo_pos;
    }
  }
  
  if (abs(last_ELservo_pos-ELservo_pos)<minELperslot)
  {
    last_ELservo_pos = ELservo_pos;
  }
  else
  {
    new_servo_pos = (last_ELservo_pos+ELservo_pos)/2;
    if (abs(new_servo_pos-ELservo_pos)>maxELperslot)
    {
      if (new_servo_pos > last_ELservo_pos)
      {
        last_ELservo_pos += maxELperslot;
      }
      else
      {
        last_ELservo_pos -= maxELperslot;
      }      
    }
    else
    {
      last_ELservo_pos = new_servo_pos;
    }
  }

  driveAZ(last_AZservo_pos);
  driveEL(last_ELservo_pos);
  
  delay(slot_time_ms);
  
  reset_timer+=slot_time_ms;
  
  if (reset_timer>reset_limit_ms)
  {
    reset_timer=0;
    AZ=0;  // Move to due North (AZ/EL 0/0)
    EL=0;
  }

}

void translateAZEL(int AZ, int EL, int *AZservo_pos, int *ELservo_pos)
{
// Convert requested AZ/EL values into servo angles.
// AZ ranges from 0-359, but our AZ servo is only capable of 0-180.
// EL ranges from 0-90.  We use the EL range from 90-180 in conjunction
// with the AZ servo to flip over and point to 'the other side'.
  //print "Requested move to %d/%d"%(AZ,EL)

  AZ=AZ%360;
  EL=EL%360;
  if (EL>90)
  {
    EL=90;
  }

  // Calculate the destination quadrant.
  // 0 (NE) 0-89 degrees
  // 1 (SE) 90-179 degrees
  // 2 (SW) 180-269 degrees
  // 3 (NW) 270-359 degrees
  int quadrant = AZ/90;  // Integer division gives us 0,1,2 or 3

  switch (quadrant)
  {
    case 0:
      (*AZservo_pos)=90-AZ;
      (*ELservo_pos)=EL;
      break;
    case 1:
      (*AZservo_pos)=270-AZ;
      (*ELservo_pos)=180-EL;
      break;
    case 2:
      (*AZservo_pos)=270-AZ;
      (*ELservo_pos)=180-EL;
      break;
    case 3:
      (*AZservo_pos)=450-AZ;
      (*ELservo_pos)=EL;
      break;
  }
  
}

void driveEL(int EL)
{
  // Drive EL servo to an actual value 0-180 degrees
  Serial.print("Driving EL to ");
  Serial.print(EL);
  Serial.print("\n");
  if (ELservo_zero_is_min)
  {
    ELservo.write(EL);  
  }
  else
  {
    ELservo.write(180-EL);
  }
}

void driveAZ(int AZ)
{
  // Drive AZ servo to an actual value 0-180 degrees
  Serial.print("Driving AZ to ");
  Serial.print(AZ);
  Serial.print("\n");
  if (AZservo_zero_is_min)
  {
    AZservo.write(AZ);  
  }
  else
  {
    AZservo.write(180-AZ);
  }
}
