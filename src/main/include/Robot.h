#pragma once

#include <Lib830.h>
#include <frc/WPILib.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <ctre/Phoenix.h>
#include "Elevator.h"
#include "Arm.h"
#include "Spear.h"
enum GearState {HIGH = false, LOW = true};

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
	void handleClimb();
	void handleSpear();
	void handleElevator();
	double drivetrainDeadzone(double);

  private:
	// Motor IDs
	static const int RIGHT_FRONT_MOTOR_ID = 1;
	static const int LEFT_FRONT_MOTOR_ID = 2;
	static const int RIGHT_BACK_MOTOR_ID = 3;
	static const int LEFT_BACK_MOTOR_ID = 4;
	static const int ARM_MOTOR_ID = 5;
	static const int ELEVATOR_MOTOR_ID = 6;
	static const int FLYWHEEL_MOTOR_ID = 7;
	static const int CLIMBER_ACTUATOR_MOTOR1_ID = 8;
	static const int CLIMBER_ACTUATOR_MOTOR2_ID = 9;
	

	static const int GYRO_ANALOG_PIN = 0;
	static const int POTENTIOMETER_ANALOG_PIN = 3;
	
	static const int UPPER_LIMIT_SWITCH_DIO = 0;
	static const int LOWER_LIMIT_SWITCH_DIO = 1;
	static const int CLIMBER_LIMIT_SWITCH_DIO = 2;

	static const int GEARSHIFT_SOLENOID_PIN = 4;
	static const int HATCH_GRAB_SOLENOID_PIN = 6;
	static const int EXTENSION_SOLENOID_PIN = 7;

	// Misc
	static const int TICKS_TO_ACCEL = 20;
	static constexpr double FLYWHEEL_THRESHOLD = 0.2;
	static constexpr double JOINT_MOVEMENT_SPEED = 0.5;
	static constexpr double CONTROLLER_GYRO_THRESHOLD = 0.2;
	static constexpr double SPEED_GYRO_THRESHOLD =0.1;
	static constexpr double DRIVETRAIN_DEADZONE_THRESHOLD = 0.2;
	static constexpr double MANUAL_ELEVATOR_THRESHOLD = 0.2;
	static constexpr double ARM_THRESHOLD = 0.2;
	static constexpr double VISION_TRIGGER_THRESHOLD = 0.3;

	// Vision Declarations
	bool doingAutoAlign = false;
	double visionSteer = 0.0;
	static constexpr double CAMERA_WIDTH = 320;
	static constexpr double VISION_TURN_FACTOR = 100.0;
	static constexpr double CARGO_TURN_FACTOR = 100.0;
	static constexpr double TARGET_WIDTH_TO_CAMERA_OFFSET_RATIO = 0.3;

	// Drivetrain declarations
  	WPI_TalonSRX rightFront {RIGHT_FRONT_MOTOR_ID};
	WPI_TalonSRX leftFront {LEFT_FRONT_MOTOR_ID};
	WPI_VictorSPX rightBack {RIGHT_BACK_MOTOR_ID};
	WPI_VictorSPX leftBack {LEFT_BACK_MOTOR_ID};
	frc::DifferentialDrive drivetrain {leftFront, rightFront};
	frc::Solenoid gearShifter{GEARSHIFT_SOLENOID_PIN};
	double prevAngle = 0;
	double prevSpeed = 0;
	double speed = 0;
	Toggle gyroCorrectOn{true};
	GearState gearState = LOW;

	// Controller declarations
	Lib830::GamepadF310 pilot {0};
	Lib830::GamepadF310 copilot {1};

	// Misc component declarations
	frc::AnalogGyro gyro {GYRO_ANALOG_PIN};

	// Rotating Arm Declarations
	WPI_VictorSPX joint{ARM_MOTOR_ID};
	WPI_VictorSPX flywheel{FLYWHEEL_MOTOR_ID};
	Toggle armManualMode{false};
	Toggle armUp{false};
	Toggle armDown{false};
	frc::AnalogPotentiometer pot{POTENTIOMETER_ANALOG_PIN, 300.0};
	Arm arm{joint, flywheel, pot};

	// Spear Declarations
	frc::Solenoid hatchGrabPiston{HATCH_GRAB_SOLENOID_PIN};
  	frc::Solenoid extensionPiston{EXTENSION_SOLENOID_PIN};
	Spear spear{hatchGrabPiston, extensionPiston};

	// Elevator Declarations
	frc::DigitalInput upperLimitSwitch{0};
	frc::DigitalInput lowerLimitSwitch{1};
	WPI_TalonSRX elevatorMotor{ELEVATOR_MOTOR_ID};
	Elevator elevator{elevatorMotor,upperLimitSwitch,lowerLimitSwitch};
	enum ElevatorMode {MANUAL = true,AUTOMATIC = false};
	ElevatorMode elevatorMode = AUTOMATIC;
	Toggle leftBumper;
	Toggle rightBumper;

	//climber declarations
	bool climbing = false;
	nt::NetworkTableEntry nt_climberAngle, nt_climberHeight;
	double climberAngle = 140;
	double climberHeight = 100;
	frc::DigitalInput climberSwitch{CLIMBER_LIMIT_SWITCH_DIO};
	WPI_TalonSRX climberActuator1{CLIMBER_ACTUATOR_MOTOR1_ID};
	WPI_TalonSRX climberActuator2{CLIMBER_ACTUATOR_MOTOR2_ID};
	frc::Timer climberTimer; 
};