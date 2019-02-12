#pragma once

#include <Lib830.h>
#include <frc/WPILib.h>
#include <ctre/Phoenix.h>
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
	double drivetrainDeadzone(double);

  private:
	// Motor IDs
	static const int RIGHT_FRONT_MOTOR_ID = 1; 
	static const int LEFT_FRONT_MOTOR_ID = 2; 
	static const int RIGHT_BACK_MOTOR_ID = 3; 
	static const int LEFT_BACK_MOTOR_ID = 4; 
	static const int ANALOG_GYRO_PIN = 0;
	static const int POTENTIOMETER_ANALOG_PIN = 3;
	static const int WINCH_MOTOR_ID = 0;
	static const int FLYWHEEL_MOTOR_ID = 0;
	static const int ELEVATOR_MOTOR_ID = 0;
	static const int GEARSHIFT_SOLENOID_PIN = 4;
	static const int HATCH_GRAB_SOLENOID_PIN = 6;
	static const int EXTENSION_SOLENOID_PIN = 7;


	// Misc
	static const int TICKS_TO_ACCEL = 10;
	static constexpr double FLYWHEEL_THRESHOLD = 0.05;
	static constexpr double JOINT_MOVEMENT_SPEED = 0.5;
	static constexpr double CONTROLLER_GYRO_THRESHOLD = 0.1;
	static constexpr double SPEED_GYRO_THRESHOLD = 0.1;
	static constexpr double DRIVETRAIN_DEADZONE_THRESHOLD = 0.05;
	static constexpr double MANUAL_ELEVATOR_THRESHOLD = 0.1;
	static constexpr double ARM_THRESHOLD = 0.1;
	static constexpr double VISION_TRIGGER_THRESHOLD = 0.3;


	// Vision Declarations
	bool doingAutoAlign = false;
	double visionSteer = 0.0;
	static constexpr double CAMERA_WIDTH = 320;
	static constexpr double TURN_SCALE_FACTOR = 15.0;

	// Drivetrain declarations
	WPI_TalonSRX rightFront {RIGHT_FRONT_MOTOR_ID};
	WPI_TalonSRX leftFront {LEFT_FRONT_MOTOR_ID};
	WPI_VictorSPX rightBack {RIGHT_BACK_MOTOR_ID};
	WPI_VictorSPX leftBack {LEFT_BACK_MOTOR_ID};
	frc::DifferentialDrive drivetrain {leftBack, rightBack};
	frc::Solenoid gearShifter{GEARSHIFT_SOLENOID_PIN};
	double prevAngle = 0;
	double prevSpeed = 0;
	double speed = 0;
	Toggle gyroCorrectState{true};
	GearState gearState = LOW;

	// Controller declarations
	Lib830::GamepadF310 pilot {0};
	Lib830::GamepadF310 copilot {1};

	// Misc component declarations
	frc::AnalogGyro gyro {ANALOG_GYRO_PIN};

	// Rotating Arm Declarations
	WPI_VictorSPX joint{WINCH_MOTOR_ID};
	WPI_VictorSPX flywheel{FLYWHEEL_MOTOR_ID};
	Toggle manualMode{true};
	Toggle armUp{false};
	Toggle armDown{false};
	frc::AnalogPotentiometer pot{POTENTIOMETER_ANALOG_PIN};
	Arm arm{joint, flywheel, pot};
	int currentArmSetpoint = 0;
	
	// Spear Declarations
	frc::Solenoid hatchGrabPiston{HATCH_GRAB_SOLENOID_PIN};
    frc::Solenoid extensionPiston{EXTENSION_SOLENOID_PIN};
	Spear spear{hatchGrabPiston, extensionPiston};

	// Elevator Declarations
	WPI_VictorSPX winch{ELEVATOR_MOTOR_ID};
	Elevator elevator{winch};
	enum ElevatorMode {MANUAL,AUTOMATIC};
	ElevatorMode elevatorMode = MANUAL;
	int currentSetpoint = 0;
	Toggle leftBumper;
	Toggle rightBumper;
};