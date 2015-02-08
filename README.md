This is the 2014 code for Space Cookies FRC Team 1868.

Contents:

AutoCommand.cpp

AutoCommand.h

AutonomousController.cpp

AutonomousController.h

ButtonReader.cpp

ButtonReader.h

CameraController.cpp

CameraController.h

ControlBoard.cpp

ControlBoard.h

Debugging.cpp

Debugging.h

DriveController.cpp

DriveController.h

DSLCDPrinter.cpp

DSLCDPrinter.h

ini.cpp

ini.h

LinearVictor.cpp

LinearVictor.h

MainProgram.cpp

minGlue.h :
  This connects the ini file to the rest of the program.

PIDControlLoop.cpp :
  PIDControlLoop.cpp defines all of the methods and contains most of the math behind an accurate PID loop.

PIDControlLoop.h :
  PIDControlLoop.h declares the methods and variables needed to enact a PID loop that we use for accuracy measurements in autonomous. This is also where the PIDConfig structure is made.

RemoteControl.h :
  This is an interface for the Control Board.
  
RobotModel.cpp :
  Defines the methods that are used by the controllers to access the different objects and values from the objects that are part of the robot like talons, solenoids, gyros and more.
  
RobotModel.h :
  Declares the methods and variables for the virtual representation of the robot. These are accessors and mutators mainly for our controllers to call. Also contains all of the objects that are part of the robot like talons, solenoids, gyros and so forth.
  
RobotPorts2013.h :
  List of last years robot ports so that when we are testing using old robots, we can easily access these ports.
  
RobotPorts2014.h :
  List of our robot ports in a convenient place so that if the ports change physically, they are easy to change in the program.
  
ShooterController.cpp :
  ShooterController.cpp defines all of the methods and in the update method, we use a state machine to control the superstructure. This way we control when the catapult is released with buttons from the control board and when the intake rollers will be turned on. 
  
ShooterController.h :
  ShooterController.h declares the methods and variables for our shooter controller. This controller controls the catapult, the intake rollers for the 2014 robot in a state machine. It relates the control board and the robot model and makes the superstructure work.
