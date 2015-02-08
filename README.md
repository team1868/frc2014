This is the 2014 code for Space Cookies FRC Team 1868.

Contents:

AutoCommand.cpp :
  AutoCommand.cpp contains the methods for autonomous and includes DriveCommand, PivotCommand, IntakeRollers, and
  IntakePositionCommand. It also has a saturate method to ensure that all motor values are less than or equal to
  the maximum value.

AutoCommand.h :
  AutoCommand.h declares the methods and variables used to drive, pivot, shoot, intake, and shoot into a hot goal.

AutonomousController.cpp :
  AutonomousController.cpp contains the list of commands to do during autonomous, in the form of a queue. It also
  has methods to start the autonomous functions, update, reset, and refresh the ini.

AutonomousController.h :
  AutonomousController.h declares the methods and variables used in AutonomousController.cpp for the autonomous
  sequence.

ButtonReader.cpp :
  ButtonReader.cpp contains all methods relating to buttons, such as what to do when a button has been pressed
  or released, or how to access the current state of a button. Contains ToggleButtonReader and SwitchReader.

ButtonReader.h :
  ButtonReader.h declares the variables and methods used for buttons, which mainly are booleans relating to the
  state of the button. Also has accessors.

CameraController.cpp :
  

CameraController.h

ControlBoard.cpp

ControlBoard.h

Debugging.cpp

Debugging.h

DriveController.cpp

DriveController.h

DSLCDPrinter.cpp :
  DSLCDPrinter.cpp takes a variable that wants to be printed to the Driver Station lower right corner and prints it there and deals with space limits and updating.

DSLCDPrinter.h :
  DSLCDPrinter.h takes a variable that wants to be printed to the Driver Station lower right corner and prints it there and deals with space limits and updating.

ini.cpp :
  ini.cpp is the definitions for the methods in ini.h that take an ini text file and read it and give our code appropriate values.

ini.h :
  ini.h is code for being able to read an ini text file that we use for on-field software configuration.

LinearVictor.cpp :
    This is code for creating accurate victors so that the inputs we give the victors will represent what the motors are doing. We do not use this in the 2014 code because we use talons.
    
LinearVictor.h :
  This is code for creating accurate victors so that the inputs we give the victors will represent what the motors are doing. We do not use this in the 2014 code because we use talons.

MainProgram.cpp :
  MainProgram.cpp is where all of the controllers are created. It inherits from Iterative Robot in WPILib. 

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
