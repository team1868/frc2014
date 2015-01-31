#include "WPILib.h"
#include "ControlBoard.h"
#include "DriveController.h"
#include "CameraController.h"
#include "AutonomousController.h"
#include "ShooterController.h"
#include "RobotModel.h"
#include "Debugging.h"
#include "DSLCDPrinter.h"


// The MainProgram that is linked into the FRC code
class MainProgram : public IterativeRobot {

public:

	RemoteController *humanControl;
	RobotModel *robot;
	CameraController *cameraController;
	AutonomousController *autoController;
	DriveController *driveController;
	ShooterController *shooterController;
	// ADXL345_I2C *accel;
	DSLCDPrinter *dsLCDPrinter;
	LiveWindow *lw;

	double currTimeSec;
	double lastTimeSec;
	double deltaTimeSec;

	MainProgram(void) {
		// constructor
		robot = new RobotModel();
		humanControl = new ControlBoard(robot);
		driveController = new DriveController(robot, humanControl);
		cameraController = new CameraController(robot, humanControl);
		shooterController = new ShooterController(robot, humanControl);
		autoController = new AutonomousController(robot, cameraController, shooterController);
		// accel = new ADXL345_I2C(1/*,DataFormat_Range(kRange_2G*/);
		dsLCDPrinter = new DSLCDPrinter(robot->dsLCD);
		dsLCDPrinter->AddLCDMessage(new RobotModelLCDMessage(robot));
		dsLCDPrinter->AddLCDMessage(new CameraControllerLCDMessage(cameraController));
		lw = LiveWindow::GetInstance();

		currTimeSec = 0.0;
		lastTimeSec = 0.0;
		deltaTimeSec = 0.0;

		SetPeriod(0.01);
	}

	void RobotInit(void) {
		//lw is live window on drive station
		lw->AddSensor("Gyro", "Gyro angle", robot->gyro);
		lw->AddSensor("Right Encoder", "Right encoder val",
				robot->rightWheelEncoder);
		lw->AddSensor("Left Encoder", "Left encoder val",
				robot->leftWheelEncoder);
		robot->EnableCompressor();
		robot->ResetGyro();
		robot->ResetEncoder(robot->leftWheelEncoder);
		robot->ResetEncoder(robot->rightWheelEncoder);
		robot->ResetTimer();
	}

	void DisabledInit(void) {
		// Actions that are performed once as soon as the robot becomes disabled
		driveController->Reset();
#ifdef USE_CAMERA
		cameraController->Reset(); // to be completed
#endif
		autoController->Reset();
		shooterController->Reset();

	}

	void AutonomousInit(void) {

		// Actions that are performed once as soon as the autonomous period begins
		RefreshAllIni();

		robot->ResetGyro();
		robot->ResetEncoder(robot->leftWheelEncoder);
		robot->ResetEncoder(robot->rightWheelEncoder);
		robot->SetWheelSpeed(RobotModel::kBothWheels, 0.0);
		robot->ResetTimer();

		autoController->Reset();
		shooterController->Reset();
#ifdef USE_CAMERA
		cameraController->Reset();
#endif
		autoController->StartAutonomous();
	}

	void TeleopInit(void) {

		// Actions that are performed once as soon as the teleop period begins	
		RefreshAllIni();

		driveController->Reset();
#ifdef USE_CAMERA
		cameraController->Reset();
#endif
		shooterController->Reset();
		robot->ResetEncoder(robot->leftWheelEncoder);
		robot->ResetEncoder(robot->rightWheelEncoder);
		robot->ResetGyro();

	}

	void StateInit(void) {
		RobotInit();
		DisabledInit();
		AutonomousInit();
		TeleopInit();
	}

	void DisabledPeriodic(void) {
		// Actions that are performed periodically while the robot is disabled
		
	}

	void AutonomousPeriodic(void) {
		
		 // Actions that are performed periodically during autonomous
		 lastTimeSec = currTimeSec;
		 currTimeSec = robot->timer->Get();
		 deltaTimeSec = currTimeSec - lastTimeSec;

		 /*
		 humanControl->ReadControls();
		 driveController->Update(currTimeSec, deltaTimeSec);
		 */
		 
		autoController->Update(currTimeSec, deltaTimeSec);
		MATCH_PERIODIC(10, dsLCDPrinter->Update());

	}
	void TeleopPeriodic(void) {

		// Actions that are performed periodically during teleop

		lastTimeSec = currTimeSec;
		currTimeSec = robot->timer->Get();
		deltaTimeSec = currTimeSec - lastTimeSec;
		
		humanControl->ReadControls();
		driveController->Update(currTimeSec, deltaTimeSec);
		shooterController->Update(currTimeSec, deltaTimeSec);
#ifdef USE_CAMERA
		cameraController->Update(currTimeSec, deltaTimeSec);
#endif
		MATCH_PERIODIC(10, dsLCDPrinter->Update());
	}
	void TestPeriodic() {
		lw->Run();
	//Actions that are performed periodically during test mode
		MATCH_PERIODIC(10, dsLCDPrinter->Update());
	}

	void RefreshAllIni() {
		robot->RefreshIni();
		enableDoPeriodic = robot->pini->geti("DEBUGGING", "enableDoPeriodic", 1);
		if (enableDoPeriodic == 1) {
			printf("DO_PERIODIC is enabled\n");
		} else printf("DO_PERIODIC is disabled\n");
		autoController->RefreshIni();
		shooterController->RefreshIni();

#ifdef USE_CAMERA
		cameraController->RefreshIni();
#endif
	}

};

START_ROBOT_CLASS(MainProgram);

