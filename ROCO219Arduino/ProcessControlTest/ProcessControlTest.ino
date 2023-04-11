///////////////////////////////////////////////////////////////
// ProcessControlTest program for inverted pendulum system
// runs on Arduino Mega
///////////////////////////////////////////////////////////////
// Author: Dr. Ian Howard
// Associate Professor (Senior Lecturer) in Computational Neuroscience
// Centre for Robotics and Neural Systems  
// Plymouth University
// A324 Portland Square 
// PL4 8AA
// Plymouth,   Devon,  UK
// howardlab.com
// 20/04/2017

///////////////////////////////////////////////////////////////
// Define functions that are called to generate desired control

// command functions activated my menu
void StopControl() {
  Serial.println("StopControl");
  // Stop Control code goes here ...
}
void ActivateControl() {
  Serial.println("ActivateControl");
  // ActivateControl Control code goes here ...
}

// Define function to print out menu options on request
// print out help commands to serial output
void PrintHelp()
{
  // print help commands
  Serial.println("*** ProcessControlTest ***");
  Serial.println("h: print help commands");
  Serial.println("v: print signal values");
  Serial.println("s: stop control");
  Serial.println("a: activate control");
  Serial.println(" ");
}

///////////////////////////////////////////////////////////////
// poll loop function to decode menu input typed into the Arduino serial monitor
void PollControlMenuCommands()
{
  // check for incoming serial data:
  if (Serial.available() > 0) {

    // read incoming serial data:
    char inChar = Serial.read();
    // print out command
    Serial.print("Received command ");
    Serial.println(inChar);

    // Use a switch function to decode the command 
    // and call the appropriate command function
    switch (inChar)
    {
      // print help commands
      case 'h':
        PrintHelp();
        break;
      case 'v':
        Serial.println("print signals");
        //  output printing lines do here....
        break;
      // stop control
      case 's':
        StopControl();
        break;
      // activate control
      case 'a':
        ActivateControl();
        break;
    }
  }
}

///////////////////////////////////////////////////////////////
// Arduino setup function
void setup() {

  // setup serial monitor
  Serial.begin (9600);

  // print help
  PrintHelp();
}

///////////////////////////////////////////////////////////////
// Arduino loop function to process commands
void loop() {
  // run the options menu
  PollControlMenuCommands();
}
