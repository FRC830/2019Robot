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
	static const int RIGHT = 0; //PLACEHOLDER!!!
	static const int LEFT = 1; //PLACEHOLDER!!!
	static const int ANALOG_GYRO = 0; //PLACEHOLDER!!!

	VictorSP right {RIGHT};
	VictorSP left {LEFT};

	DifferentialDrive drivetrain {left, right};			
	AnalogGyro gyro {ANALOG_GYRO};
  	//Differential Drive!
};
