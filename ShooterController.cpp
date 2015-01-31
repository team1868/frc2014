#include "ShooterController.h"
#include "ControlBoard.h"
#include "RobotPorts2014.h"
#include "WPILib.h"
#include <math.h>

/**
 * Constructor, initializes all variables to null/0 and sets the robot and 
 * driver station values
 */
ShooterController::ShooterController(RobotModel* myRobot,
		RemoteController* myHumanControl) {
	robot = myRobot;
	humanControl = myHumanControl;

	shootingMotorSpeed = 0.0;
	armingMotorSpeed = 0.0;
	intakeMotorSpeed = 0.0;
	
	shootingDuration = 0;
	shootingCounter = 0;
	
	waitingDuration = 0;
	waitingCounter = 0;
	
	latchingDuration = 0;
	latchingCounter = 0;
	
	softShotLowGearDuration = 0;
	softShotLowGearCounter = 0;
	softShotHighGearDuration = 0;
	softShotHighGearCounter = 0;
	
	armingInProgress = false;
	autoShootRequested = false;

	useTestMode = 0;
	testMotorSpeed = 0.0;
	
	debugFrequency = 10;

	intakeStateDesired = kNothingDesired;
	
	autoIntakeRollersRequestedOn = false;
	
#ifdef NEWBOT
	m_stateVal = kInit;
#endif
}

/**
 * Update method, called by Teleop Periodic
 * States:
 * 		kInit: Initializes all of the counters to zero, executes on the first
 * 		time the Update method is called
 * 		kIdle: Idle state, listens for arming and shooting buttons, will 
 * 		probably put intake code in here
 * 		kShooting: 
 * 			Can be entered using the shooting button
 * 			On the first iteration, it simply shifts the gearbox to high gear; 
 * 			On the second iteration, it releases the latch and sets the motor
 * 			speed to the one specified in the ini file
 * 			On every subsequent iteration, it continues to set the motor speed
 * 			nextState = kWaiting
 * 		kWaiting: Simply increases a counter, it is a wait in between the shoot
 * 		and auto-arm to make sure that the arm is all the way forward before we
 * 		begin to drive it back down
 * 		nextState = kArming
 * 		kArming: 
 * 			Can be entered using the arming button or from kWaiting
 * 			On the first iteration, we make sure that the latch is released, 
 * 			then shift to low gear, and set the catapult motor speed
 * 			On all subsequent iterations, continues to set catapult motor speed
 * 			if limit switch is not pressed
 * 			If limit switch is pressed, exit state
 * 			nextState = kLatching
 * 		kLatching: Can be entered from kArming, fires latch and continues driving motors
 * 		until time is up, nextState = kArmingDone
 * 		kArmingDone: Can be entered after kLatching, nextState = kIdle
 * 		Default: Only entered if something goes wrong with the state machine, should never be
 * 		entered
 */
void ShooterController::Update(double currTimeSec, double deltaTimeSec) {
#ifdef NEWBOT
	switch (m_stateVal) {
	case (kInit):
		shootingCounter = 0;
		waitingCounter = 0;
		latchingCounter = 0;
		softShotLowGearCounter = 0;
		softShotHighGearCounter = 0;
		armingInProgress = false;
		
		robot->LowerIntakeArmIn(robot->GetLowerIntakePiston()->Get());
		robot->UpperIntakeArmIn(robot->GetUpperIntakePiston()->Get());
		
		bool lowerIntakePos = robot->IsLowerIntakeIn(); // solenoids
		bool upperIntakePos = robot->IsUpperIntakeIn();
		
		if (lowerIntakePos && upperIntakePos) {
			intakePosition = kHoldBallPos;
		} else if (lowerIntakePos && !upperIntakePos) {
			intakePosition = kIntakingPos;
		} else if (!lowerIntakePos && upperIntakePos) {
			intakePosition = kShootingPos;
		} else {
			intakePosition = kStartingPos;
		}
		
		nextState = kIdle;
		break;

	case (kIdle):
		
		nextState = kIdle;
	
		if (humanControl->IntakingDesired() || (intakeStateDesired == kIntakingDesired)) {
			robot->MoveLowerIntakeArmIn();
			robot->MoveUpperIntakeArmOut();
			DO_PERIODIC(1, printf("Intake in intaking pos\n"));
			intakePosition = kIntakingPos;
			intakeStateDesired = kNothingDesired;
		}
		
		if (humanControl->HoldBallDesired() || (intakeStateDesired == kHoldingDesired)) {
			robot->MoveLowerIntakeArmIn();
			robot->MoveUpperIntakeArmIn();
			DO_PERIODIC(1, printf("Intake in holding pos\n"));
			intakePosition = kHoldBallPos;
			intakeStateDesired = kNothingDesired;
		} 
		
		if (humanControl->IntakeStartDesired() || (intakeStateDesired == kStartingDesired)) {
			robot->MoveLowerIntakeArmOut();
			robot->MoveUpperIntakeArmOut();
			DO_PERIODIC(1, printf("Intake in starting pos\n"));
			intakePosition = kStartingPos;
			intakeStateDesired = kNothingDesired;
		}
		
		if (humanControl->IntakeShootPositionDesired() || (intakeStateDesired == kShootingDesired)) {
			robot->MoveLowerIntakeArmOut();
			robot->MoveUpperIntakeArmIn();
			DO_PERIODIC(1, printf("Intake in shooting pos\n"));
			intakePosition = kShootingPos;
			intakeStateDesired = kNothingDesired;
		}
		
		if (humanControl->PopBallPistonInDesired()) {
			robot->MovePopBallPistonIn();
		}
		
		if (humanControl->PopBallPistonOutDesired()) {
			robot->MovePopBallPistonOut();
		}
	
		if (humanControl->ShooterDesired() || autoShootRequested) {
			if (useTestMode == 0) {
				nextState = kShooting;
			} else {
				nextState = kTest;
			}
			
		}
		
		if (humanControl->ArmingDesired()) {
			nextState = kArming;
		} 
		
		if (humanControl->IntakeDesiredForward() || 
				autoIntakeRollersRequestedOn) {
			DO_PERIODIC(debugFrequency, printf("Drive intake roller forward\n"));
			robot->SetIntakeSpeed(intakeMotorSpeed);
		} else if (humanControl->IntakeDesiredBackward()) {
			DO_PERIODIC(debugFrequency, printf("Drive intake roller backward\n"));
			robot->SetIntakeSpeed(-intakeMotorSpeed);
		} else {
			robot->SetIntakeSpeed(0.0);
		}
		break;
	
	case (kShooting):
		if (intakePosition == kHoldBallPos) {
			robot->MoveLowerIntakeArmIn();
			robot->MoveUpperIntakeArmOut();
			DO_PERIODIC(1, printf("Intake in intaking pos\n"));
			intakePosition = kIntakingPos;
			Wait(0.1);
		} else if (intakePosition == kStartingPos) {
			robot->MoveLowerIntakeArmOut();
			robot->MoveUpperIntakeArmIn();
			DO_PERIODIC(1, printf("Intake in shooting pos\n"));
			intakePosition = kShootingPos;
			Wait(0.1);
		}
		
		
			// Make motor speed positive so that the motor runs counter-clockwise
		if (shootingCounter == 0) {
			robot->ShiftCatapultToHighGear();
			DO_PERIODIC(1, printf("Shifting catapult to high gear\n"));
			DO_PERIODIC(1, printf("Raw catapult encoder value: %ld\n",
							robot->catapultEncoder->GetRaw()));
		} else if (shootingCounter == 1) {
			robot->ReleaseLatch();//don't want to break chain, release latch first
			robot->SetCatapultMotorSpeed(shootingMotorSpeed);
			DO_PERIODIC(1, printf("Firing piston to deactivate latch and shoot\n"));
			DO_PERIODIC(1, printf("Raw catapult encoder value: %ld\n",
							robot->catapultEncoder->GetRaw()));
		} else {
			robot->SetCatapultMotorSpeed(shootingMotorSpeed);
			DO_PERIODIC(debugFrequency, printf("Raw catapult encoder value: %ld\n",
							robot->catapultEncoder->GetRaw()));
			DO_PERIODIC(debugFrequency,
					printf("Waiting for physical system to finish shooting\n"));
		}

		if (shootingCounter < shootingDuration) {
			shootingCounter += 1;
			nextState = kShooting;
		} else {
			robot->EngageLatch();
			robot->SetCatapultMotorSpeed(0.0);
			shootingCounter = 0;
			nextState = kWaiting;
		}

		break;
		
	
	case (kWaiting):
		DO_PERIODIC(debugFrequency, printf("Waiting\n"));
		if (waitingCounter < waitingDuration) {
			waitingCounter += 1;
			nextState = kWaiting;
		} else {
			waitingCounter = 0;
			nextState = kArming;
		}
		break;
			
	case (kArming):
		// Make motor speed negative so that the motor runs clockwise
		if (!armingInProgress) {
			robot->EngageLatch(); // Make sure latch is released
			robot->ShiftCatapultToLowGear();
			DO_PERIODIC(1, printf("Shifting to low gear\n"));
			robot->SetCatapultMotorSpeed(armingMotorSpeed); 
			DO_PERIODIC(1, printf("Driving catapult arm down\n"));
			DO_PERIODIC(1, printf("Raw catapult encoder value: %ld\n", 
								robot->catapultEncoder->GetRaw()));
			armingInProgress = true;
			nextState = kArming;
		} else if (!ArmingDone()) {
			robot->SetCatapultMotorSpeed(armingMotorSpeed);
			DO_PERIODIC(debugFrequency, printf("Driving catapult arm down\n"));
			DO_PERIODIC(debugFrequency, printf("Raw catapult encoder value: %ld\n", 
								robot->catapultEncoder->GetRaw()));
			nextState = kArming;
		} else {
			robot->SetCatapultMotorSpeed(0.0);
			DO_PERIODIC(1, printf("Raw catapult encoder value: %ld\n", 
								robot->catapultEncoder->GetRaw()));
			DO_PERIODIC(1, printf("Catapult arm all the way down\n"));
			armingInProgress = false;
			nextState = kLatching;
		}
		break;

	case (kLatching):
		if (latchingCounter == 0) {
			robot->SetCatapultMotorSpeed(0.0);
			DO_PERIODIC(1, printf("Waiting to latch\n"));
		} else {
			DO_PERIODIC(debugFrequency,
					printf("Waiting for physical system to finish latching\n"));
		}

		if (latchingCounter < latchingDuration) {
			latchingCounter += 1;
			nextState = kLatching;
		} else {
			DO_PERIODIC(1, printf("Raw catapult encoder value: %ld\n", 
								robot->catapultEncoder->GetRaw()));
			DO_PERIODIC(1, printf("Finished latching\n"));
			latchingCounter = 0;
			nextState = kArmingDone;
		}
		break;

	case (kArmingDone):
		/*Instead of shifting catapult to high gear, we are going to leave it
		 *in low gear so that if the latch for some reason accidentally 
		 *releases, the arm will not unexpectedly go flying up
		 */ 
		DO_PERIODIC(1, printf("Arming done\n"));
		autoShootRequested = false;
		nextState = kIdle;
		break;
		
	case (kTest):
		if (softShotLowGearCounter < softShotLowGearDuration) {
			DO_PERIODIC(5, printf("Inside soft shot low gear\n"));
			if (softShotLowGearCounter == 0) {
				robot->ShiftCatapultToLowGear();
				robot->ReleaseLatch();
				robot->SetCatapultMotorSpeed(testMotorSpeed);robot->SetCatapultMotorSpeed(0.0);
				softShotLowGearCounter += 1;
				nextState = kTest;
			} else {
				robot->SetCatapultMotorSpeed(testMotorSpeed);
				softShotLowGearCounter += 1;
				nextState = kTest;
			}
		} else if (softShotHighGearCounter < softShotHighGearDuration) {
			DO_PERIODIC(5, printf("Inside soft shot high gear\n"));
			if (softShotHighGearCounter == 0) {
				robot->SetCatapultMotorSpeed(0.0);
				robot->ShiftCatapultToHighGear();
			}
			softShotHighGearCounter += 1;
			nextState = kTest;
		} else {
			softShotLowGearCounter = 0;
			softShotHighGearCounter = 0;
			nextState = kIdle;
		}
		
		break;
	
	default:
		printf("Should never come here\n");
		nextState = kIdle;
		break;

	}

	m_stateVal = nextState;
#endif
}
/**
 * Called from Teleop init, resets the counters and other variables
 */
void ShooterController::Reset() {
	shootingCounter = 0;
	waitingCounter = 0;
	latchingCounter = 0;
	softShotLowGearCounter = 0;
	softShotHighGearCounter = 0;
	
	armingInProgress = false;
	intakeStateDesired = kNothingDesired;
	autoIntakeRollersRequestedOn = false;
#ifdef NEWBOT
	m_stateVal = kInit;
	robot->SetCatapultMotorSpeed(0.0);
	robot->SetIntakeSpeed(0.0);
#endif
}

/**
 * Refreshes all of the ini file values and stores them in variables
 */
void ShooterController::RefreshIni() {
	shootingMotorSpeed = robot->pini->getf("CATAPULT", "shootingMotorSpeed", 0.9);
	armingMotorSpeed = robot->pini->getf("CATAPULT",  "armingMotorSpeed", -0.9);
	intakeMotorSpeed = robot->pini->getf("CATAPULT", "intakeMotorSpeed", 0.5);
	
	shootingDuration = robot->pini->geti("CATAPULT", "shootingDuration", 100);
	shootDelayDuration = robot->pini->geti("CATAPULT", "shootDelayDuration", 50);
	waitingDuration = robot->pini->geti("CATAPULT", "waitingDuration", 40);
	latchingDuration = robot->pini->geti("CATAPULT", "latchingDuration", 100);
	
	useTestMode = robot->pini->geti("CATAPULT", "useTestMode", 0);
	testMotorSpeed = robot->pini->getf("CATAPULT", "testMotorSpeed", 0.7);
	softShotLowGearDuration = robot->pini->geti("CATAPULT", "softShotLowGearDuration", 10);
	softShotHighGearDuration = robot->pini->geti("CATAPULT", "softShotHighGearDuration", 40);
}

/**
 * Method to request shooting from autonomous
 */
void ShooterController::RequestAutoShoot() {
	autoShootRequested = true;
}

/**
 * The opposite of an is done method, it returns true if the robot is still
 * shooting, the variable is set to false in kArmingDone
 */
bool ShooterController::AutoShootInProgress() {
	return autoShootRequested;
}

void ShooterController::RequestIntakePos(uint32_t intakeState) {
	intakeStateDesired = intakeState;
}

void ShooterController::RequestAutoIntakeRollers(bool on){
	if (on) {
		autoIntakeRollersRequestedOn = true;
	}
	else {
		autoIntakeRollersRequestedOn = false;
	}
}
/**
 * Called from kArming, it checks if the limit switch is press to see if the 
 * robot is done arming
 */
bool ShooterController::ArmingDone() {
	if (robot->limitSwitch->Get() == 1) {
		return true;
	} else {
		return false;
	}
}

/**
 * Destructor
 */
ShooterController::~ShooterController() {

}
