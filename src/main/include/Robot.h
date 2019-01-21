#pragma once

#include <string>
#include <Lib830.h>
#include <frc/WPILib.h>
#include <thread>
#include <opencv2/core/core.hpp>
#include "GripPipeline.h"
#include "Elevator.h"
#include "Arm.h"
#include <vector>

class Robot : public frc::TimedRobot {
public:
	void RobotInit() override;
	void RobotPeriodic() override;
	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
	void TestPeriodic() override;
	void handleJoint();
	void handleDrivetrain();
	void handleFlywheel();
	void handlePistons();
	void handleElevator();
	static void CameraLoop();

  private:
	// Motor Pins
	static const int RIGHT_FRONT_MOTOR_PIN = 0; 
	static const int LEFT_FRONT_MOTOR_PIN = 0; 
	static const int RIGHT_BACK_MOTOR_PIN = 0; 
	static const int LEFT_BACK_MOTOR_PIN = 0; 
	static const int ANALOG_GYRO_PIN = 0;
	static const int POTENTIOMETER_ANALOG_PIN = 0;
	static const int WINCH_MOTOR_PIN = 0;
	static const int FLYWHEEL_MOTOR_PIN = 0;
	static const int ELEVATOR_MOTOR_PIN = 0;
	static const int SOLENOID_PIN = 0;
	// Encoder Stuff
	static const int ENCODER_TICKS = 1024;
	static constexpr float PI = 3.1415927;
	static const int WINCH_DIAMETER = 6; // PLACEHOOLDER;
	static const int ENCODER_TICK_DISTANCE = 6 * PI / ENCODER_TICKS;
	// Encoder Pins
	static const int ELEVATOR_ENCODER_DIO_ONE = 999;
	static const int ELEVATOR_ENCODER_DIO_TWO = 998;
	static const int DRIVETRAIN_ENCODER_LEFT_DIO_ONE = 991;
	static const int DRIVETRAIN_ENCODER_LEFT_DIO_TWO = 992;
	static const int DRIVETRAIN_ENCODER_RIGHT_DIO_ONE = 993;
	static const int DRIVETRAIN_ENCODER_RIGHT_DIO_TWO = 994;
	// Constants
	static const int TICKS_TO_ACCEL = 10;
	static constexpr double FLYWHEEL_THRESHOLD = 0.05;
	// Variables
	double prevAngle = 0; 
	double prevSpeed = 0;
	double speed = 0;
	bool bumperPressed = false;	
	
	// Drivetrain declarations
	frc::VictorSP rightFront {RIGHT_FRONT_MOTOR_PIN};
	frc::VictorSP leftFront {LEFT_FRONT_MOTOR_PIN};
	frc::VictorSP rightBack {RIGHT_BACK_MOTOR_PIN};
	frc::VictorSP leftBack {LEFT_BACK_MOTOR_PIN};
	frc::SpeedControllerGroup left {leftFront, leftBack};
	frc::SpeedControllerGroup right {rightFront, rightBack};
	frc::DifferentialDrive drivetrain {left, right};	
	// Control declarations
	Lib830::GamepadF310 pilot {0};
	Lib830::GamepadF310 copilot {1};
	//misc component declarations
	frc::AnalogGyro gyro {ANALOG_GYRO_PIN};
	// Arm Declarations

	frc::VictorSP joint{WINCH_MOTOR_PIN};
	frc::VictorSP flywheel{FLYWHEEL_MOTOR_PIN};
	frc::AnalogPotentiometer pot{POTENTIOMETER_ANALOG_PIN};
	frc::Solenoid piston{SOLENOID_PIN};
	Arm arm{joint, flywheel, pot, piston};
	std::vector<double> heights = {10.0, 20.0, 30.0, 40.0, 50.0, 60.0};
	double currentHeight = heights[0];
	// Elevator Declarations
	frc::VictorSP winch{ELEVATOR_MOTOR_PIN};
	frc::Encoder elevatorEncoder{ELEVATOR_ENCODER_DIO_ONE, ELEVATOR_ENCODER_DIO_TWO};
	Elevator elevator{winch, elevatorEncoder};
};
