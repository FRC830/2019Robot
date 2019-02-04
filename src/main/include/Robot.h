#pragma once

#include <string>
#include <Lib830.h>
#include <frc/WPILib.h>
#include <ctre/Phoenix.h>
#include <thread>
#include <opencv2/core/core.hpp>
#include <vector>
#include "Elevator.h"
#include "Arm.h"
#include "Spear.h"

class Robot : public frc::TimedRobot {
public:
	void RobotInit() override;
	void RobotPeriodic() override;
	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
	void TestPeriodic() override;
	void DisabledInit() override;
	void handleVision();
	void handleArm();
	void handleDrivetrain();
	void handleFlywheel();
	void handleSpear();
	void handleElevator();
	double deadzone(double);

  private:
	// Motor IDs
	static const int RIGHT_FRONT_MOTOR_ID = 1; 
	static const int LEFT_FRONT_MOTOR_ID = 2; 
	static const int RIGHT_BACK_MOTOR_ID = 3; 
	static const int LEFT_BACK_MOTOR_ID = 4; 
	static const int ANALOG_GYRO_PIN = 0;
	static const int POTENTIOMETER_ANALOG_PIN = 0;
	static const int WINCH_MOTOR_ID = 0;
	static const int FLYWHEEL_MOTOR_ID = 0;
	static const int ELEVATOR_MOTOR_ID = 0;
	static const int GEARSHIFT_SOLENOID_PIN = 5;
	static const int HATCH_GRAB_SOLENOID_PIN = 6;
	static const int EXTENSION_SOLENOID_PIN = 7;

	// Encoder Values
	static const int ENCODER_TICKS = 1024;
	static constexpr double PI = 3.1415927;
	static const int WINCH_DIAMETER = 6; // PLACEHOOLDER;
	static const int ENCODER_TICK_DISTANCE = 6 * PI / ENCODER_TICKS;

	// Encoder Pins
	static const int ELEVATOR_ENCODER_DIO_ONE = 0;
	static const int ELEVATOR_ENCODER_DIO_TWO = 0;
	static const int DRIVETRAIN_ENCODER_LEFT_DIO_ONE = 0;
	static const int DRIVETRAIN_ENCODER_LEFT_DIO_TWO = 0;
	static const int DRIVETRAIN_ENCODER_RIGHT_DIO_ONE = 0;
	static const int DRIVETRAIN_ENCODER_RIGHT_DIO_TWO = 0;

	// Misc
	static const int TICKS_TO_ACCEL = 10;
	static constexpr double FLYWHEEL_THRESHOLD = 0.05;
	static constexpr double JOINT_MOVEMENT_SPEED = 2.0;
	static constexpr double CONTROLLER_GYRO_THRESHOLD = 0.1;
	double prevAngle = 0; 
	double prevSpeed = 0;
	double speed = 0;
	bool bumperPressed = false;
	bool gyroCorrectEnabled = false;
	std::vector<double> heights = {10.0, 20.0, 30.0, 40.0, 50.0, 60.0};
	double currentHeight = heights[0];
	

	//Vision
	bool doingAutoAlign = false;
	double visionSteer = 0.0;

	double prevLeftRectArea = -1;
	double prevRightRectArea = -1;
	double prevTargetMidpoint = -1;
	
	double currentLeftRectArea = -1;
	double currentRightRectArea = -1;
	double currentTargetMidpoint = -1;

	double combineIndividualPrevAndCurrentData(double prev, double current);
	std::vector<double> combinePrevAndCurrentVisionData();

	void doFullDataVision(std::vector<double> data);
	void doMidpointOnlyVision(double midpoint);

	// Drivetrain declarations
	WPI_VictorSPX rightFront {RIGHT_FRONT_MOTOR_ID};
	WPI_VictorSPX leftFront {LEFT_FRONT_MOTOR_ID};
	WPI_TalonSRX rightBack {RIGHT_BACK_MOTOR_ID};
	WPI_TalonSRX leftBack {LEFT_BACK_MOTOR_ID};
	frc::SpeedControllerGroup left {leftFront, leftBack};
	frc::SpeedControllerGroup right {rightFront, rightBack};
	frc::DifferentialDrive drivetrain {left, right};
	frc::Solenoid gearshifter{GEARSHIFT_SOLENOID_PIN};

	// Control declarations
	Lib830::GamepadF310 pilot {0};
	Lib830::GamepadF310 copilot {1};

	// Misc component declarations
	frc::AnalogGyro gyro {ANALOG_GYRO_PIN};

	// Arm Declarations
	WPI_VictorSPX joint{WINCH_MOTOR_ID};
	WPI_VictorSPX flywheel{FLYWHEEL_MOTOR_ID};
	frc::AnalogPotentiometer pot{POTENTIOMETER_ANALOG_PIN};
	Arm arm{joint, flywheel, pot};
	
	// Spear Declarations
	frc::Solenoid hatchGrabPiston{HATCH_GRAB_SOLENOID_PIN};
    frc::Solenoid extensionPiston{EXTENSION_SOLENOID_PIN};
	Spear spear{hatchGrabPiston, extensionPiston};

	// Elevator Declarations
	WPI_VictorSPX winch{ELEVATOR_MOTOR_ID};
	frc::Encoder elevatorEncoder{ELEVATOR_ENCODER_DIO_ONE, ELEVATOR_ENCODER_DIO_TWO};
	Elevator elevator{winch, elevatorEncoder};
};
