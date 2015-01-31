#include "AutoCommand.h"
#include "PIDControlLoop.h"
#include "RobotModel.h"

#include <math.h>
#include "Debugging.h"
#include <iostream>
#include <string>

using namespace std;

double Saturate(double value, double maxAbsValue) {
/* 
 returns the value with the smaller absolute value
 - used to make sure that all motor outputs are less than or equal to the maximum value
 */
	if (value >= 0.0) {
		return min(value, maxAbsValue);
	} else {
		return max(value, -maxAbsValue);
	}
}
DriveCommand::DriveCommand(double myDistance, double maxSpeed,
		RobotModel* myRobot) 
	: SimpleAutoCommand() {
	robot = myRobot;
	desiredAutoDistance = myDistance;
	maxAutoSpeed = maxSpeed;
	autoDriveState = kAutoDriveStart;
	autoDriveDone = false;
	initialAngle = 0.0;
	leftEncoderInitialValue = 0.0;
	rightEncoderInitialValue = 0.0;
}
/*
 initializing the static variables so that they can be accessed in Autonomous 
 Controller so they can be refreshed from the ini file
 */
double DriveCommand::drivePFac = 0.0;
double DriveCommand::driveIFac = 0.0;
double DriveCommand::driveDFac = 0.0;
double DriveCommand::gyroPFac = 0.0;
double DriveCommand::gyroIFac = 0.0;
double DriveCommand::gyroDFac = 0.0;


void DriveCommand::Init() {
	printf("just requested auto drive from auto controller\n");
	drivePFac = robot->pini->getf("DrivePID", "PFAC", 0.2);
	driveIFac = robot->pini->getf("DrivePID", "IFAC", 0.001);
	driveDFac = robot->pini->getf("DrivePID", "DFAC", 4.0);

	gyroPFac = robot->pini->getf("DrivePID", "GYRO_PFAC", 0.1);
	gyroIFac = robot->pini->getf("DrivePID", "GYRO_IFAC", 0.0);
	gyroDFac = robot->pini->getf("DrivePID", "GYRO_DFAC", 0.0);
	
	autoDriveState = kAutoDriveStart;
	autoDriveDone = false;
/*
 Getting the initial sensor values for the PID Loops
 */
	leftEncoderInitialValue = robot->GetWheelEncoderDistance(RobotModel::kLeftWheel);
	rightEncoderInitialValue = robot->GetWheelEncoderDistance(RobotModel::kRightWheel);
	initialAutoDistance = DistanceDriven();  // this is the current encoder value
	initialAngle = robot->GetGyroAngle();
/*
 Creating the PID Loops with the initial sensor values to calculate the motor values
 in order to go specific distances
*/
	PIDConfig* pidDriveDistanceConfig = CreateDriveDistanceConfig();
	pidDriveDistance = new PIDControlLoop(pidDriveDistanceConfig);
	pidDriveDistance->Init(initialAutoDistance, initialAutoDistance + desiredAutoDistance);
	PIDConfig* pidDriveStraightConfig = CreateDriveStraightConfig();
	pidDriveStraight = new PIDControlLoop(pidDriveStraightConfig);	
	pidDriveStraight->Init(initialAngle, initialAngle);
}

void DriveCommand::Update(double currTimeSec, double deltaTimeSec) {
	double currentAutoDistance = 0.0;
	string currState = " ";

	switch (autoDriveState) {
	case (kAutoDriveStart):
		currState = "kAutoDriveStart";
		initialAutoDistance = robot->GetWheelEncoderDistance(
				RobotModel::kBothWheels);
		printf("initial distance of encoder: %f\n", initialAutoDistance);
		printf("current state is kAutoDriveStart\n");
		robot->SetWheelSpeed(RobotModel::kBothWheels, maxAutoSpeed);
		nextState = kAutoDrive;
		break;

	case (kAutoDrive):
		currState = "kAutoDrive";
		currentAutoDistance = DistanceDriven();
		DO_PERIODIC(20, printf("Distance travelled in auto drive %f\n", currentAutoDistance));
		
		if (pidDriveDistance->ControlLoopDone(currentAutoDistance)) {
			nextState = kAutoDriveDone; //Used to be kAutoPivot, but might no be necessary on the new robot
		} else {
			/*
			 Get the motor speeds from the corollating PID Loops and factoring in the
			 output from the PID Loop for driving straight
			 */
			double pSpeed = pidDriveDistance->Update(currentAutoDistance);
			double angleSpeed = (DriveStraight()) / 2.0;
			double adjustedMaxAutoSpeed = fabs(maxAutoSpeed - fabs(angleSpeed));
			pSpeed = Saturate(pSpeed, adjustedMaxAutoSpeed);
			DO_PERIODIC(20, printf("pSpeed after saturation: %f\n", pSpeed));

			robot->SetWheelSpeed(RobotModel::kLeftWheel, pSpeed + angleSpeed);
			robot->SetWheelSpeed(RobotModel::kRightWheel, pSpeed - angleSpeed);
			DO_PERIODIC(20, printf("left wheel speed is %f\n", (pSpeed + angleSpeed)));
			DO_PERIODIC(20, printf("right wheel speed is %f\n", (pSpeed - angleSpeed)));

			nextState = kAutoDrive;
		}

		break;

	case (kAutoPivot):
		currState = "kAutoPivot";
		printf("in Auto Pivot");
		double angleSpeed = (DriveStraight()) / 2.0;
		float newAngle = robot->GetGyroAngle();
		if (pidDriveStraight->ControlLoopDone(newAngle)) {
			nextState = kAutoDriveDone;
		/*
		float newError = initialAngle - newAngle;
		if (newError < 5.0 && newError > -5.0) {
			nextState = kAutoDriveDone;
		*/
		} else {
			robot->SetWheelSpeed(RobotModel::kLeftWheel, angleSpeed);
			robot->SetWheelSpeed(RobotModel::kRightWheel, -angleSpeed);
			nextState = kAutoPivot;
		}
	
		break;

	case (kAutoDriveDone):
		currState = "kAutoDriveDone";
		printf("Auto Drive Done so setting wheel speed to 0.0\n");
		robot->SetWheelSpeed(RobotModel::kBothWheels, 0.0);
		autoDriveDone = true;
		nextState = kAutoDriveDone;
		break;
	}

	autoDriveState = nextState;
}

bool DriveCommand::IsDone() {
	return autoDriveDone;
}

double DriveCommand::DriveStraight() {
	/*
	 Gets the motor output value for driving straight from the PID Loop
	 */
	double currAngle = robot->GetGyroAngle();
	double speed = pidDriveStraight->Update(currAngle);
	speed = Saturate(speed, maxAutoSpeed);
	return speed;
}

double DriveCommand::DistanceDriven() {
	/*
	 Returns the average distance driven from the two encoders if both encoders are
	 giving accurate values. Find out if any of the encoder values are too far 
	 away from each other
	 */
	double leftEncoderValue = robot->GetWheelEncoderDistance(RobotModel::kLeftWheel);
	double rightEncoderValue = robot->GetWheelEncoderDistance(RobotModel::kRightWheel);
	double leftDriven = leftEncoderValue - leftEncoderInitialValue;
	double rightDriven = rightEncoderValue - rightEncoderInitialValue;
	double distanceAccuracy = max(0.2, 0.1 * max(fabs(leftDriven), fabs(rightDriven)));
	if (fabs(leftDriven - rightDriven) < distanceAccuracy) {
		DO_PERIODIC(5, printf("Encoder's fine \n"));
		return (leftDriven + rightDriven)/2.0;
	} else if (fabs(leftDriven) > fabs(rightDriven)) {
		DO_PERIODIC(5, printf("Taking left distance \n"));
		return leftDriven;
	} else {
		DO_PERIODIC(5, printf("Taking right distance \n"));
		return rightDriven;
	}
	
}

PIDConfig* DriveCommand::CreateDriveDistanceConfig() {
	PIDConfig* pidConfig = new PIDConfig();
	pidConfig->pFac = drivePFac;
	pidConfig->iFac = driveIFac;
	pidConfig->dFac = driveDFac;
	pidConfig->desiredAccuracy = 0.25;
	pidConfig->maxAbsDiffError = 1.0;
	pidConfig->maxAbsError = 3.0;
	pidConfig->maxAbsOutput = maxAutoSpeed;
	pidConfig->maxAbsITerm = 0.2;
	return pidConfig;
}

PIDConfig* DriveCommand::CreateDriveStraightConfig() {
	PIDConfig* pidConfig = new PIDConfig();
	pidConfig->pFac = gyroPFac;
	pidConfig->iFac = gyroIFac;
	pidConfig->dFac = gyroDFac;
	pidConfig->desiredAccuracy = 0.1;
	pidConfig->maxAbsDiffError = 1.0;
	pidConfig->maxAbsError = 0.0;
	pidConfig->maxAbsOutput = maxAutoSpeed;
	pidConfig->maxAbsITerm = 1.0;
	return pidConfig;
}

PivotCommand::PivotCommand(double myAngle, double maxSpeed, RobotModel* myRobot)
	: SimpleAutoCommand() {
	desiredAngle = myAngle;
	maxPivotSpeed = maxSpeed;
	robot = myRobot;
	isDone = false;
}

/*
 initializing the static variables so that they can be accessed in Autonomous 
 Controller so they can be refreshed from the ini file
 */

double PivotCommand::pFac = 0.0;
double PivotCommand::iFac = 0.0;
double PivotCommand::dFac = 0.0;
double PivotCommand::desiredAccuracy = 0.0;

void PivotCommand::Init() {
	printf("Just initialized PivotCommand\n");
	double initialAngle = robot->GetGyroAngle();

	pFac = robot->pini->getf("PivotPID", "PFAC", 0.5);
	iFac = robot->pini->getf("PivotPID", "IFAC", 0.0);
	dFac = robot->pini->getf("PivotPID", "DFAC", 1.0);
	desiredAccuracy = robot->pini->getf("PivotPID", "DesiredAccuracy", 5.0);
	isDone = false;
	/*
	 Creating the PID Loop for pivoting using the initial angle.
	 The PID loop is used to calculate the motor output needed for the robot to 
	 turn a certain amount
	 */
	PIDConfig* pidConfig = CreatePivotConfig();
	pidPivot = new PIDControlLoop(pidConfig);
	pidPivot->Init(initialAngle, desiredAngle);
}

void PivotCommand::Update(double currTimeSec, double deltaTimeSec) {
	/*
	 Get the motor output value from the PID loop and set the motor speeds
	 */
	double currentAngle = robot->GetGyroAngle();
	double speed = pidPivot->Update(currentAngle);
	isDone = pidPivot->ControlLoopDone(currentAngle);
	if (isDone) {
		speed = 0.0;
	}
	robot->SetWheelSpeed(RobotModel::kLeftWheel, speed / 2.0);
	robot->SetWheelSpeed(RobotModel::kRightWheel, -speed / 2.0);
}

bool PivotCommand::IsDone() {
	return isDone;
}

PIDConfig* PivotCommand::CreatePivotConfig() {
	PIDConfig* pidConfig = new PIDConfig();
	pidConfig->pFac = pFac;
	pidConfig->iFac = iFac;
	pidConfig->dFac = dFac;
	pidConfig->desiredAccuracy = desiredAccuracy;
	pidConfig->maxAbsDiffError = 1.0;
	pidConfig->maxAbsError = 0.0;
	pidConfig->maxAbsOutput = maxPivotSpeed;
	pidConfig->maxAbsITerm = 1.0;
	return pidConfig;
}

void ShootingCommand::Init() {
	shooterController->RequestAutoShoot();
}

void ShootingCommand::Update(double currTimeSec, double deltaTimeSec) {
	shooterController->Update(currTimeSec, deltaTimeSec);
}

bool ShootingCommand::IsDone() {
	if (!shooterController->AutoShootInProgress()) {
		printf("Shooting Done /n");
		return true;
	} else {
		return false;
	}
}

void IntakeRollersCommand::Init() {
	timer->Start();
	shooterController->RequestAutoIntakeRollers(true); //turn on rollers
}

void IntakeRollersCommand::Update(double currTimeSec, double deltaTimeSec) {
	if (timer->Get() >= waitTimeSec) {
		shooterController->RequestAutoIntakeRollers(false); //if time is up, turn off rollers
		rollersAreDone = true;
	}
	shooterController->Update(currTimeSec, deltaTimeSec);
}
bool IntakeRollersCommand::IsDone() {
	return rollersAreDone;
}

/*
 initializing the static variables so that they can be accessed in Autonomous 
 Controller so they can be refreshed from the ini file
 */

double IntakePositionCommand::intakeWaitTime = 0.0;

IntakePositionCommand::IntakePositionCommand(ShooterController* myShooterController, uint32_t intakePosition) {
	shooterController = myShooterController;
	intakePosDesired = intakePosition;
	timer = new Timer();
}

void IntakePositionCommand::Init() {
	timer->Start();
	shooterController->RequestIntakePos(intakePosDesired);
	DO_PERIODIC(1, printf("IntakePositionCommand initializing\n"));
	int posDesired = (int)intakePosDesired;
	DO_PERIODIC(1, printf("IntakePosDesired %i", posDesired));
}

void IntakePositionCommand::Update(double currTimeSec, double deltaTimeSec) {
	shooterController->Update(currTimeSec, deltaTimeSec);
	DO_PERIODIC(1, printf("IntakePositionCommand updating\n"));
}

bool IntakePositionCommand::IsDone() {
	if (timer->Get() < 0.3) {
		return false;
	} else {
		DO_PERIODIC(1, printf("IntakePositionCommand is done\n"));
		return true;
	}
}
