#include "WPILib.h"
#include "RobotModel.h"

#include <math.h>

RobotModel::RobotModel() {

	pini = new Ini("/robot.ini");
#ifdef NEWBOT
	driveLeftA = new Talon(LEFT_MOTOR_A_PWM_PORT); //constant from RobotPorts2014
	driveLeftB = new Talon(LEFT_MOTOR_B_PWM_PORT);
	driveRightA = new Talon(RIGHT_MOTOR_A_PWM_PORT);
	driveRightB = new Talon(RIGHT_MOTOR_B_PWM_PORT);
	catapultMotorA = new Talon(CATAPULT_MOTOR_A_PWM_PORT);
	catapultMotorB = new Talon(CATAPULT_MOTOR_B_PWM_PORT);
	intakeMotorA = new Talon(INTAKE_MOTOR_A_PWM_PORT);
	intakeMotorB = new Talon(INTAKE_MOTOR_B_PWM_PORT);

	isLowerIntakeIn = false;
	isUpperIntakeIn = true;
#else
	driveLeftA = new Victor(LEFT_MOTOR_A_PWM_PORT); //constant from RobotPorts2014
	driveLeftB = new Victor(LEFT_MOTOR_B_PWM_PORT);
	driveRightA = new Victor(RIGHT_MOTOR_A_PWM_PORT);
	driveRightB = new Victor(RIGHT_MOTOR_B_PWM_PORT);
#endif

	leftWheelEncoder = new Encoder(LEFT_WHEEL_ENCODER_A_PWM_PORT,
			LEFT_WHEEL_ENCODER_B_PWM_PORT, true);
	rightWheelEncoder = new Encoder(RIGHT_WHEEL_ENCODER_A_PWM_PORT,
			RIGHT_WHEEL_ENCODER_B_PWM_PORT, true);
	catapultEncoder = new Encoder(CATAPULT_ENCODER_A_PWM_PORT,
			CATAPULT_ENCODER_B_PWM_PORT, true);

	timer = new Timer();
	timer->Start();

	leftWheelEncoder->SetDistancePerPulse((PI / 3) / 256); // 4 inch wheels, 12 inches/ft, 256 tics per rotation
	rightWheelEncoder->SetDistancePerPulse(-(PI / 3) / 256);

	leftWheelEncoder->Start();
	rightWheelEncoder->Start();
	catapultEncoder->Start();
	isLowGear = true;

	compressor = new Compressor(COMPRESSOR_PRESSURE_SWITCH_CHAN,
			COMPRESSOR_RELAY_CHAN);

	dsLCD = DriverStationLCD::GetInstance();

	gyro = new Gyro(GYRO_PORT);
	gyro->Reset();
	gyro->SetSensitivity(.007);	// used to be .0033, last year used .007
	
	limitSwitch = new DigitalInput(LIMIT_SWITCH_PWM_PORT);
	opticalSensor = new DigitalInput(OPTICAL_SENSOR_PWM_PORT);
	
	shifterSolenoid = new Solenoid(SHIFTER_SOLENOID_CHAN);
#ifdef NEWBOT
	catapultShifterSolenoid = new Solenoid(CATAPULT_SHIFTER_SOLENOID_CHAN);
	latchSolenoid = new Solenoid(LATCH_SOLENOID_CHAN);
	popBallSolenoid = new Solenoid(POP_BALL_SOLENOID_CHAN);
	intakeUpperArmSolenoidA = new Solenoid(INTAKE_UPPER_ARM_SOLENOID_CHAN_A);
	intakeUpperArmSolenoidB = new Solenoid(INTAKE_UPPER_ARM_SOLENOID_CHAN_B);
	intakeLowerArmSolenoidA = new Solenoid(INTAKE_LOWER_ARM_SOLENOID_CHAN_A);
	intakeLowerArmSolenoidB = new Solenoid(INTAKE_LOWER_ARM_SOLENOID_CHAN_B);
#endif
#ifdef USE_CAMERA
	hasCamera = false; //use to see if camera is causing a problem
	camera = NULL;
	camera = &(AxisCamera::GetInstance("10.18.68.11"));
	if (camera == NULL) {
		hasCamera = false;
	} else {
		hasCamera = true;
		camera->WriteResolution(AxisCamera::kResolution_640x480);
		camera->WriteCompression(20);
		camera->WriteBrightness(30);
	}
#endif
}

void RobotModel::SetWheelSpeed(Wheels w, double speed) {
	switch (w) {
	case kLeftWheel:
		driveLeftA->Set(speed);
		driveLeftB->Set(speed);
		break;
	case kRightWheel:
		driveRightA->Set(-speed);
		driveRightB->Set(-speed);
		break;
	default:
	case kBothWheels:
		driveLeftA->Set(speed);
		driveLeftB->Set(speed);
		driveRightA->Set(-speed);
		driveRightB->Set(-speed);
	}
}

bool RobotModel::OpticalSensorActivated() {
	if (opticalSensor->Get() == 0){
		return true;
	}
	else return false;
}

#ifdef USE_CAMERA
HSLImage* RobotModel::GetCameraImage() {
	if (camera == NULL) {
		return NULL;
	} else {
		return camera->GetImage();
	}
}
#endif

void RobotModel::EnableCompressor() {
	compressor->Start();
}

void RobotModel::DisableCompressor() {
	compressor->Stop();
}

bool RobotModel::GetCompressorState() {
	return (compressor->Enabled());
}


void RobotModel::ResetGyro()  {
	gyro->Reset();
}

float RobotModel::GetGyroAngle() {
	return gyro->GetAngle();
}

double RobotModel::GetWheelEncoderDistance(Wheels w) {
	switch (w) {
	case kLeftWheel:
		return leftWheelEncoder->GetDistance();
	case kRightWheel:
		return rightWheelEncoder->GetDistance();
	default:
	case kBothWheels:
		return ((leftWheelEncoder->GetDistance()
				+ rightWheelEncoder->GetDistance()) / 2);
	}
}

double RobotModel::GetLeftWheelEncoderSpeed() {
	return leftWheelEncoder->GetRate();
}

double RobotModel::GetRightWheelEncoderSpeed() {
	return rightWheelEncoder->GetRate();
}

double RobotModel::GetWheelEncoderSpeed() {
	//average of both encoders
	return ((GetLeftWheelEncoderSpeed() + GetRightWheelEncoderSpeed()) / 2);
}

void RobotModel::ResetEncoder(Encoder *e) {
	e->Reset();
}

bool RobotModel::IsLowGear() {
	return isLowGear;
}

void RobotModel::ShiftToHighGear() {
	shifterSolenoid->Set(true);
	isLowGear = false;
}

void RobotModel::ShiftToLowGear() {
	shifterSolenoid->Set(false);
	isLowGear = true;
}

void RobotModel::ShiftGear() {
	if (IsLowGear()) {
		ShiftToHighGear();
	} else
		ShiftToLowGear();
}
#ifdef NEWBOT
void RobotModel::ShiftCatapultToHighGear() {
	catapultShifterSolenoid->Set(true);
}

void RobotModel::ShiftCatapultToLowGear() {
	catapultShifterSolenoid->Set(false);
}

void RobotModel::EngageLatch() {
	latchSolenoid->Set(false);
}

void RobotModel::ReleaseLatch() {
	latchSolenoid->Set(true);
}

#endif
void RobotModel::ResetTimer() {
	timer->Reset();
}
#ifdef USE_CAMERA
bool RobotModel::CheckCameraConnection() {
	return false;
}
#endif

void RobotModel::RefreshIni() {
	delete pini;
	pini = new Ini("/robot.ini");
}

#ifdef NEWBOT
void RobotModel::SetCatapultMotorSpeed(double speed) {
	catapultMotorA->Set(speed);
	catapultMotorB->Set(speed);
}

bool RobotModel::IsLowerIntakeIn() {
	return isLowerIntakeIn;
}

void RobotModel::LowerIntakeArmIn(bool in) {
	isLowerIntakeIn = in;
}

// returns the solenoid that is true when the piston is in
Solenoid* RobotModel::GetLowerIntakePiston() {
	return intakeLowerArmSolenoidA;
}

void RobotModel::MoveLowerIntakeArmIn() {
	intakeLowerArmSolenoidA->Set(true);
	intakeLowerArmSolenoidB->Set(false);
	isLowerIntakeIn = true;
}

void RobotModel::MoveLowerIntakeArmOut() {
	intakeLowerArmSolenoidA->Set(false);
	intakeLowerArmSolenoidB->Set(true);
	isLowerIntakeIn = false;
}

bool RobotModel::IsUpperIntakeIn() {
	return isUpperIntakeIn;
}

// returns the solenoid that is true when the piston is in
void RobotModel::UpperIntakeArmIn(bool in) {
	isUpperIntakeIn = in;
}

Solenoid* RobotModel::GetUpperIntakePiston() {
	return intakeUpperArmSolenoidA;
}

void RobotModel::MoveUpperIntakeArmIn() {
	intakeUpperArmSolenoidA->Set(true);
	intakeUpperArmSolenoidB->Set(false);
	isUpperIntakeIn = true;
}

void RobotModel::MoveUpperIntakeArmOut() {
	intakeUpperArmSolenoidA->Set(false);
	intakeUpperArmSolenoidB->Set(true);
	isUpperIntakeIn = false;
}

Solenoid* RobotModel::GetPopBallPiston() {
	return popBallSolenoid;
}

void RobotModel::MovePopBallPistonIn() {
	popBallSolenoid->Set(true);
}

void RobotModel::MovePopBallPistonOut() {
	popBallSolenoid->Set(false);
}

void RobotModel::SetIntakeSpeed(double speed) {
	intakeMotorA->Set(speed);
	intakeMotorB->Set(speed);
}

#endif

RobotModel::~RobotModel() {
	delete driveLeftA;
	delete driveLeftB;
	delete driveRightA;
	delete driveRightB;

#ifdef NEWBOT
	delete catapultMotorA;
	delete catapultMotorB;
#endif
}
