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
	static const int RIGHT = 0; //PLACEHOLDER!!!
	static const int LEFT = 1; //PLACEHOLDER!!!
	static const int ANALOG_GYRO = 0; //PLACEHOLDER!!!

	frc::VictorSP right {RIGHT};
	frc::VictorSP left {LEFT};

	frc::DifferentialDrive drivetrain {left, right};			
	frc::AnalogGyro gyro {ANALOG_GYRO};
  	//Differential Drive!
};
