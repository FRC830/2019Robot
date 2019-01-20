#pragma once

#include <string>
#include <Lib830.h>
#include <frc/WPILib.h>
#include <thread>
#include <opencv2/core/core.hpp>
#include "GripPipeline.h"

class Robot : public frc::TimedRobot {
public:
	void RobotInit() override;
	void RobotPeriodic() override;
	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
	void TestPeriodic() override;
	static void CameraLoop();

private:

	// Motor Pins
	static const int RIGHT_MOTOR_PORT = 0; 
	static const int LEFT_MOTOR_PORT = 0; 
	static const int ANALOG_GYRO_PORT = 0;
	static const int POTENTIOMETER_ANALOG_PORT = 0;
	static const int WINCH_MOTOR_PORT = 0;
	static const int FLYWHEEL_MOTOR_PORT = 0;
	static const int ELEVATOR_MOTOR_PORT = 0;
	// Encoder Pins
	static const int ELEVATOR_ENCODER_DIO_ONE = 999;
	static const int ELEVATOR_ENCODER_DIO_TWO = 998;
	static const int DRIVETRAIN_ENCODER_LEFT_DIO_ONE = 991;
	static const int DRIVETRAIN_ENCODER_LEFT_DIO_TWO = 992;
	static const int DRIVETRAIN_ENCODER_RIGHT_DIO_ONE = 993;
	static const int DRIVETRAIN_ENCODER_RIGHT_DIO_TWO = 994;
	

	// Constants
	static const int TICKS_TO_ACCEL = 10;
	// Variables

	double prevAngle = 0; 
	double prevSpeed = 0;
	double speed = 0;
	
	// Drivetrain declarations
	frc::VictorSP right {RIGHT_MOTOR_PORT};
	frc::VictorSP left {LEFT_MOTOR_PORT};

	frc::DifferentialDrive drivetrain {left, right};			

	//control scheme declarations
	frc::XboxController pilot {0};

	static const frc::GenericHID::JoystickHand LEFTSIDE = frc::GenericHID::kLeftHand;
	static const frc::GenericHID::JoystickHand RIGHTSIDE = frc::GenericHID::kRightHand;

	//misc component declarations
	frc::AnalogGyro gyro {ANALOG_GYRO_PORT};

};
