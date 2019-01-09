#pragma once

#include <string>
#include <Lib830.h>
#include <SmartDashboard/SendableChooser.h>
#include <TimedRobot.h>
#include <thread>

class Robot : public frc::TimedRobot {
public:
	void RobotInit() override;
	void RobotPeriodic() override;
	void AutonomousInit() override;
	void AutonomousPeriodicov() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
	void TestPeriodic() override;
	static void CameraPeriodic();

private:
	static const int RIGHT = 0; //PLACEHOLDER!!!
	static const int LEFT = 1; //PLACEHOLDER!!!

	VictorSP right {RIGHT};
	VictorSP left {LEFT};

	DifferentialDrive drivetrain {left, right};			
		
  //Differential Drive!
  

 UsbCamera webcamfront {}
};
