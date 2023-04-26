///////////////////////////////////////////////////////////////
// SimpleHallSensor program for inverted pendulum system
// runs on Arduino Mega
// operates with SunFounder Analog Hall Switch Sensor Module
///////////////////////////////////////////////////////////////
// Author: Dr. Ian Howard
// Associate Professor (Senior Lecturer) in Computational Neuroscience
// Centre for Robotics and Neural Systems 
// Plymouth University
// A324 Portland Square
// PL4 8AA
// Plymouth,  Devon, UK
// howardlab.com
// 20/04/2017

/////////////////////////////////////////////////////////////
// Define digital inputs control pins for Hall end stop sensors 
// Use pins that support interrupts
int HallSensorLeft = 2;
int HallSensorRight = 3;

// flag to indicate if control running
volatile bool controlRunnning;

// define atop control can call running flags etc
void StopControl()
{
  // message
  Serial.println("Shut down control");
  controlRunnning = false;
}

// Left sensor callback function
void HallLeftCB   (void)
{
  // print message
  Serial.println("Left Hall Sensor activated");
  // call function to deactivate control
  StopControl();
}

// Right sensor callback function
void HallRightCB   (void)
{
  // print message
  Serial.println("Right Hall Sensor activated");
  // call function to deactivate control
  StopControl();
}

// thsi function associates callback functions with interrupt pins
void SetupHall()
{
  // Setup callback input for Left Hall sensor
  pinMode(HallSensorLeft, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HallSensorLeft), HallLeftCB, FALLING );

  // Setup callback input for Right Hall sensor
  pinMode(HallSensorRight, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HallSensorRight), HallRightCB, FALLING );
}

///////////////////////////////////////////////////////////////
// Arduino setup function 
void setup() {
  // setup serial monitor
  Serial.begin (9600);

  // setup Hall sensor callbacks
   SetupHall();

  // set enable processing glag true
  // to start with control running
  controlRunnning = true;
}

///////////////////////////////////////////////////////////////
// Arduino loop function 
void loop() {
  // poll loop runs only if controlRunnning flag active
  if (controlRunnning == false) {
    return;
  }

  // real active polling functionality would go here
  Serial.println("Running control");

  // 1s  delay so dont print too many outputs messages
  delay(1000);
}
