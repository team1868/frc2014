#include "AutonomousController.h"
#include <math.h>
#include "Debugging.h"
#include <iostream>
#include <string>
#include "AutoCommand.h"

AutonomousController::AutonomousController(RobotModel *myRobot,
		CameraController *myCamera, ShooterController *myShooter)
	: firstCommand(NULL), currentCommand(NULL), robot(myRobot), 
	  cameraController(myCamera), shooterController(myShooter) {

}

void AutonomousController::CreateQueue() {
	firstCommand = NULL;
	printf("AutoMode: %i \n", autoMode);

	switch (autoMode) {

	case (kTestMode):
		/*
		 area for testing code
		 */

	default:
	case (kBlankAuto):
		/*
		 auto that does nothing
		 */
	break;
	
	case (kSimpleDriveAuto):
		/*
		 auto that drives forward after waiting a certain amount of done
		 */
		WaitingCommand* simpleWaitCmd = new WaitingCommand(holdBallWait);
		firstCommand = simpleWaitCmd;
		DriveCommand* simpleDriveCmd = new DriveCommand(forwardDist, maxAutoSpeed, robot);
		simpleWaitCmd->SetNextCommand(simpleDriveCmd);
	break;
	
	case (kSimpleShootAuto):
			/*
			 auto that drives forward and shoots
			 */
		IntakePositionCommand* simpleShootIntakeHold =
				new IntakePositionCommand(shooterController, kHoldingDesired);
		firstCommand = simpleShootIntakeHold;
		DriveCommand* simpleShootDriveCmd = new DriveCommand(forwardDist,
				maxAutoSpeed, robot);
		simpleShootIntakeHold->SetNextCommand(simpleShootDriveCmd);
		IntakeRollersCommand* simpleShootHoldIntake =
				new IntakeRollersCommand(shooterController,
						holdBallIntakingTime);
		simpleShootDriveCmd->SetNextCommand(simpleShootHoldIntake);
		ShootingCommand* simpleShootShootCmd = new ShootingCommand(
				shooterController);
		simpleShootHoldIntake->SetNextCommand(simpleShootShootCmd);
		IntakePositionCommand* simpleShootIntakeSafe =
				new IntakePositionCommand(shooterController, kStartingDesired);
		simpleShootShootCmd->SetNextCommand(simpleShootIntakeSafe);
	break;
	
	case (kHotAuto):
			/*
			 auto that shoots according to the hot goals detected by the optical sensor
			 */
		printf("In OpticalHotAuto");
	OpticalHotShootCommand* opticalHotShootCmd1 =
				new OpticalHotShootCommand(robot);
		firstCommand = opticalHotShootCmd1;
		IntakePositionCommand* coldOpticalIntakeHold1 =
				new IntakePositionCommand(shooterController, kHoldingDesired);
		IntakePositionCommand* hotOpticalIntakeHold1 =
				new IntakePositionCommand(shooterController, kHoldingDesired);
		opticalHotShootCmd1->SetTrueNextCommand(hotOpticalIntakeHold1);
		WaitingCommand* coldOpticalDelayWait1 = new WaitingCommand(0.1);
		opticalHotShootCmd1->SetFalseNextCommand(coldOpticalDelayWait1);
		OpticalHotShootCommand* opticalHotShootCmd2 =
				new OpticalHotShootCommand(robot);
		coldOpticalDelayWait1->SetNextCommand(opticalHotShootCmd2);
		WaitingCommand* coldOpticalDelayWait2 = new WaitingCommand(0.1);
		opticalHotShootCmd2->SetTrueNextCommand(hotOpticalIntakeHold1);
		opticalHotShootCmd2->SetFalseNextCommand(coldOpticalDelayWait2);
		OpticalHotShootCommand* opticalHotShootCmd3 =
				new OpticalHotShootCommand(robot);
		coldOpticalDelayWait2->SetNextCommand(opticalHotShootCmd3);
		WaitingCommand* coldOpticalDelayWait3 = new WaitingCommand(0.1);
		opticalHotShootCmd3->SetTrueNextCommand(hotOpticalIntakeHold1);
		opticalHotShootCmd3->SetFalseNextCommand(coldOpticalDelayWait3);
		OpticalHotShootCommand* opticalHotShootCmd4 =
				new OpticalHotShootCommand(robot);
		coldOpticalDelayWait3->SetNextCommand(opticalHotShootCmd4);
		WaitingCommand* coldOpticalDelayWait4 = new WaitingCommand(0.1);
		opticalHotShootCmd4->SetTrueNextCommand(hotOpticalIntakeHold1);
		opticalHotShootCmd4->SetFalseNextCommand(coldOpticalDelayWait4);
		OpticalHotShootCommand* opticalHotShootCmd5 =
				new OpticalHotShootCommand(robot);
		coldOpticalDelayWait4->SetNextCommand(opticalHotShootCmd5);
		opticalHotShootCmd5->SetTrueNextCommand(hotOpticalIntakeHold1);
		opticalHotShootCmd5->SetFalseNextCommand(coldOpticalIntakeHold1);

		DriveCommand* coldOpticalDrive1 = new DriveCommand(forwardDist,
				maxAutoSpeed, robot);
		coldOpticalIntakeHold1->SetNextCommand(coldOpticalDrive1);
		IntakeRollersCommand* coldOpticalIntaking1 = new IntakeRollersCommand(shooterController, holdBallIntakingTime);
		coldOpticalDrive1->SetNextCommand(coldOpticalIntaking1);
		IntakePositionCommand* coldOpticalIntakeShoot1 =
				new IntakePositionCommand(shooterController, kIntakingDesired);
		coldOpticalIntaking1->SetNextCommand(coldOpticalIntakeShoot1);
		WaitingCommand* coldOpticalWait1 = new WaitingCommand(
				preShootWait + notHotWait);
		DO_PERIODIC(10, printf("waiting cold"));
		coldOpticalIntakeShoot1->SetNextCommand(coldOpticalWait1);
		ShootingCommand* opticalShootCmd1 = new ShootingCommand(
				shooterController);
		coldOpticalWait1->SetNextCommand(opticalShootCmd1);

		DriveCommand* hotOpticalDrive1 = new DriveCommand(forwardDist,
				maxAutoSpeed, robot);
		hotOpticalIntakeHold1->SetNextCommand(hotOpticalDrive1);
		IntakeRollersCommand* hotOpticalIntaking1 = new IntakeRollersCommand(shooterController, holdBallIntakingTime);
		hotOpticalDrive1->SetNextCommand(hotOpticalIntaking1);
		IntakePositionCommand* hotOpticalIntakeShoot1 =
				new IntakePositionCommand(shooterController, kIntakingDesired);
		hotOpticalIntaking1->SetNextCommand(hotOpticalIntakeShoot1);
		WaitingCommand* hotOpticalWait1 = new WaitingCommand(preShootWait);
		DO_PERIODIC(10, printf("waiting hot"));
		hotOpticalIntakeShoot1->SetNextCommand(hotOpticalWait1);
		hotOpticalWait1->SetNextCommand(opticalShootCmd1);
		IntakePositionCommand* opticalIntakeSafe1 = new IntakePositionCommand(
				shooterController, kStartingDesired);
		opticalShootCmd1->SetNextCommand(opticalIntakeSafe1);

		break;
	case (kCameraAuto):
			/*
			 auto that takes into account hot goals that are detected by the camera
			 */
#ifdef USE_CAMERA
		DriveCommand* cameraDriveCmd = new DriveCommand(forwardDist, maxAutoSpeed, robot);
		firstCommand = cameraDriveCmd;
		RequestImageCommand* requestImageCommand = new RequestImageCommand(cameraController);
		cameraDriveCmd->SetNextCommand(requestImageCommand);
		HotShootCommand* hotShootCommand = new HotShootCommand(cameraController);
		requestImageCommand->SetNextCommand(hotShootCommand);
		//If goal is hot
		WaitingCommand* hotCameraWait = new WaitingCommand(preShootWait);
		hotShootCommand->SetTrueNextCommand(hotCameraWait);
		IntakePositionCommand* hotCamIntakeShoot = new IntakePositionCommand(shooterController, kStartingDesired);
		hotCameraWait->SetNextCommand(hotCamIntakeShoot);
		ShootingCommand* hotCameraShoot = new ShootingCommand(shooterController);
		hotCamIntakeShoot->SetNextCommand(hotCameraShoot);
		// If goal is not hot
		WaitingCommand* coldCameraWait = new WaitingCommand(preShootWait + notHotWait);
		hotShootCommand->SetFalseNextCommand(coldCameraWait);
		IntakePositionCommand* coldCamIntakeShoot = new IntakePositionCommand(shooterController, kStartingDesired);
		coldCameraWait->SetNextCommand(coldCamIntakeShoot);
		ShootingCommand* coldCameraShoot = new ShootingCommand(shooterController);
		coldCamIntakeShoot->SetNextCommand(coldCameraShoot);
		
#endif
		break;
		
	case (kTwoBallAuto):
			/* hold ball Intake Position
			 * Drive Forward
			 * intaking Intake Position
			 * Waiting for settle
			 * Shoot!
			 * Drive Back
			 * Intake
			 * hold ball Intake Position
			 * Drive Forward
			 * intaking Intake Position
			 * Waiting for settle
			 * Shoot!
			 */
		IntakePositionCommand* twoBallIntakeHold1 = new IntakePositionCommand(shooterController, kHoldingDesired);
		firstCommand = twoBallIntakeHold1;
		IntakeRollersCommand* twoBallHoldIntakeCmd = new IntakeRollersCommand(shooterController,holdBallIntakingTime);
		twoBallIntakeHold1->SetNextCommand(twoBallHoldIntakeCmd);
		DriveCommand* twoBallDriveForward1 = new DriveCommand(forwardDist, maxAutoSpeed, robot);
		twoBallHoldIntakeCmd->SetNextCommand(twoBallDriveForward1);
		IntakePositionCommand* twoBallIntakeShoot1 = new IntakePositionCommand(shooterController, kIntakingDesired);
		twoBallDriveForward1->SetNextCommand(twoBallIntakeShoot1);
		WaitingCommand* twoBallWait1 = new WaitingCommand(preShootWait);
		twoBallIntakeShoot1->SetNextCommand(twoBallWait1);
		ShootingCommand* twoBallShoot1 = new ShootingCommand(shooterController);
		twoBallWait1->SetNextCommand(twoBallShoot1);
		DriveCommand* twoBallDriveBack = new DriveCommand(-forwardDist, maxAutoSpeed, robot);
		twoBallShoot1->SetNextCommand(twoBallDriveBack);
		IntakeRollersCommand* twoBallIntake = new IntakeRollersCommand(shooterController, twoBallIntakingTime);
		twoBallDriveBack->SetNextCommand(twoBallIntake);
		IntakePositionCommand* twoBallIntakeHold2 = new IntakePositionCommand(shooterController, kHoldingDesired);
		twoBallIntake->SetNextCommand(twoBallIntakeHold2);
		DriveCommand* twoBallDriveForward2 = new DriveCommand(forwardDist, maxAutoSpeed, robot);
		twoBallIntakeHold2->SetNextCommand(twoBallDriveForward2);
		IntakePositionCommand* twoBallIntakeShoot2 = new IntakePositionCommand(shooterController, kIntakingDesired);
		twoBallDriveForward2->SetNextCommand(twoBallIntakeShoot2);
		WaitingCommand* twoBallWait2 = new WaitingCommand(preShootWait);
		twoBallIntakeShoot2->SetNextCommand(twoBallWait2);
		ShootingCommand* twoBallShoot2 = new ShootingCommand(shooterController);
		twoBallWait2->SetNextCommand(twoBallShoot2);
		IntakePositionCommand* twoBallIntakeSafe = new IntakePositionCommand(shooterController, kStartingDesired);
		twoBallShoot2->SetNextCommand(twoBallIntakeSafe);
	break;
	}
}

void AutonomousController::StartAutonomous() {
	/*
	 * Set an int to the first position in the command sequence. Start the sequence.
	 */

	CreateQueue();
	currentCommand = firstCommand;
	if (currentCommand != NULL) {
		currentCommand->Init();
	}
}

void AutonomousController::Update(double currTimeSec, double deltaTimeSec) {
	/* Iterate through the command sequence.
	 * When the command you are at is done, start the next command.
	 * Continue till your position is equal to the size of the command sequence.
	 */
	if (currentCommand != NULL){
		if (currentCommand->IsDone()) {
			DO_PERIODIC(1, printf("Command Complete at: %f \n", currTimeSec));
			currentCommand = currentCommand->GetNextCommand();
			if (currentCommand != NULL) {
				currentCommand->Init();
			}
		} else {
			currentCommand->Update(currTimeSec, deltaTimeSec);
		}
	} else {
		DO_PERIODIC(100, printf("Queue finished at: %f \n", currTimeSec));
	}
}

void AutonomousController::Reset() {
	firstCommand = NULL;
	currentCommand = NULL;
	robot->SetWheelSpeed(RobotModel::kBothWheels, 0.0);
}

void AutonomousController::RefreshIni() {
	forwardDist = robot->pini->getf("AUTONOMOUS", "ForwardDist", 7.0);
	maxAutoSpeed = robot->pini->getf("AUTONOMOUS", "MaxAutoSpeed", 0.5);
	initialWait = (robot->pini)->getf("AUTONOMOUS", "initialWaitTime", 1.01);
	preShootWait = robot->pini->getf("AUTONOMOUS", "PreShootWait", 0.5);
	notHotWait = robot->pini->getf("AUTONOMOUS", "NotHotWait", 5.0);
	holdBallWait = robot->pini->getf("AUTONOMOUS", "HoldBallWait", 0.0);
	pauseBetween = robot->pini->getf("AUTONOMOUS", "PauseBetween", 2.01); 
	autoMode = robot->pini->geti("AUTONOMOUS", "AutoMode", kHotAuto); //kHotAuto = 4
	twoBallIntakingTime = robot->pini->getf("AUTONOMOUS", "TwoBallIntakingTime", 1.5);
	holdBallIntakingTime = robot->pini->getf("AUTONOMOUS", "HoldBallIntakingTime", 0.1);
	PivotCommand::pFac = robot->pini->getf("PivotPID", "PFAC", 0.5);
	PivotCommand::iFac = robot->pini->getf("PivotPID", "IFAC", 0.0);
	PivotCommand::dFac = robot->pini->getf("PivotPID", "DFAC", 1.0);
	PivotCommand::desiredAccuracy = robot->pini->getf("PivotPID", "DesiredAccuracy", 5.0);
	DriveCommand::drivePFac = robot->pini->getf("DrivePID", "PFAC", 0.2);
	DriveCommand::driveIFac = robot->pini->getf("DrivePID", "IFAC", 0.001);
	DriveCommand::driveDFac = robot->pini->getf("DrivePID", "DFAC", 4.0);
	DriveCommand::gyroPFac = robot->pini->getf("DrivePID", "GYRO_PFAC", 0.1);
	DriveCommand::gyroIFac = robot->pini->getf("DrivePID", "GYRO_IFAC", 0.0);
	DriveCommand::gyroDFac = robot->pini->getf("DrivePID", "GYRO_DFAC", 0.0);
	IntakePositionCommand::intakeWaitTime = robot->pini->getf("AUTONOMOUS", "IntakeWaitTime", 0.5);
}

AutonomousController::~AutonomousController() {
}
