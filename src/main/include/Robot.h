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

enum GearState {HIGH = true, LOW = false};

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
	void handleCargoIntake();
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
	static const int ENCODER_TICKS = 4096;
	static constexpr double PI = 3.1415927;
	static const int WINCH_DIAMETER = 6;
	static constexpr double ENCODER_TICK_DISTANCE = 6 * PI / ENCODER_TICKS;


	// Misc
	static const int TICKS_TO_ACCEL = 10;
	static constexpr double FLYWHEEL_THRESHOLD = 0.05;
	static constexpr double JOINT_MOVEMENT_SPEED = 2.0;
	static constexpr double CONTROLLER_GYRO_THRESHOLD = 0.1;
	static constexpr double CONTROLLER_DEADZONE_THRESHOLD = 0.05;
	double prevAngle = 0; 
	double prevSpeed = 0;
	double speed = 0;
	Toggle gyroCorrectState{true};
	GearState gearState = HIGH;

	//Vision
	bool doingAutoAlign = false;
	double visionSteer = 0.0;

	double prevLeftArea = -1;
	double prevRightArea = -1;
	double prevTargetMidpoint = -1;
	
	double currentLeftArea = -1;
	double currentRightArea = -1;
	double currentTargetMidpoint = -1;

	double combineIndividualPrevAndCurrentData(double prev, double current);
	std::vector<double> combinePrevAndCurrentVisionData();

	double estimateHorizontalDisplacement(double leftArea, double rightArea, double targetMidpoint);
	void doFullDataVision(std::vector<double> data);
	void doMidpointOnlyVision(double midpoint);


	// Drivetrain declarations
	WPI_VictorSPX rightFront {RIGHT_FRONT_MOTOR_ID};
	WPI_VictorSPX leftFront {LEFT_FRONT_MOTOR_ID};
	WPI_TalonSRX rightBack {RIGHT_BACK_MOTOR_ID};
	WPI_TalonSRX leftBack {LEFT_BACK_MOTOR_ID};
	frc::DifferentialDrive drivetrain {leftBack, rightBack};
	frc::Solenoid gearShifter{GEARSHIFT_SOLENOID_PIN};

	// Controller declarations
	Lib830::GamepadF310 pilot {0};
	Lib830::GamepadF310 copilot {1};

	// Misc component declarations
	frc::AnalogGyro gyro {ANALOG_GYRO_PIN};

	// Rotating Arm Declarations
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
	Elevator elevator{winch};
	double currentHeight = 0;
	Toggle leftBumper;
	Toggle rightBumper;
};