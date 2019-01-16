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

	//port and tool declarations
	static const int RIGHT = 0; 
	static const int LEFT = 1; 
	static const int ANALOG_GYRO_PORT = 0;
	static const int TICKS_TO_ACCEL = 10;
	double prevAngle = 0; 
	double prevSpeed = 0;

	
	//drivetrain declarations
	frc::VictorSP right {RIGHT};
	frc::VictorSP left {LEFT};

	frc::DifferentialDrive drivetrain {left, right};			

	//control scheme declarations
	frc::XboxController pilot {0};

	static const frc::GenericHID::JoystickHand LEFTSIDE = frc::GenericHID::kLeftHand;
	static const frc::GenericHID::JoystickHand RIGHTSIDE = frc::GenericHID::kRightHand;

	//misc component declarations
	frc::AnalogGyro gyro {ANALOG_GYRO_PORT};

};
