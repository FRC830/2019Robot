#pragma once

#include <string>
#include <Lib830.h>
#include <SmartDashboard/SendableChooser.h>
#include <TimedRobot.h>
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
	//port and tool declarations
	static const int RIGHT = 0; 
	static const int LEFT = 1; 
	static const int ANALOG_GYRO_PORT = 0;
	static const int TICKS_TO_ACCEL = 10;
	double prevAngle = 0; 
	double prevSpeed = 0;

	
	//drivetrain declarations
	VictorSP right {RIGHT};
	VictorSP left {LEFT};

	DifferentialDrive drivetrain {left, right};			

	//control scheme declarations
	XboxController pilot {0};

	static const GenericHID::JoystickHand LEFTSIDE = GenericHID::kLeftHand;
	static const GenericHID::JoystickHand RIGHTSIDE = GenericHID::kRightHand;

	//misc component declarations
	frc::AnalogGyro gyro {ANALOG_GYRO_PORT};

};
