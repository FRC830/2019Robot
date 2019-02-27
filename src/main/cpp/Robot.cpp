 /*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <Robot.h>
#include <math.h> //For trig for vision

using namespace frc;
using namespace Lib830;

// Called Initially on Robot Start
void Robot::RobotInit() {

    // Reset Motors
    rightFront.ConfigFactoryDefault();
    rightBack.ConfigFactoryDefault();
    leftFront.ConfigFactoryDefault();
    leftBack.ConfigFactoryDefault();

    // Set Victors to follow Talons
    rightBack.Follow(rightFront);
    leftBack.Follow(leftFront);
    // For if we need to change it later
    rightFront.SetSensorPhase(false);
    leftFront.SetSensorPhase(false);

    rightFront.SetInverted(false);
    rightBack.SetInverted(false);
    leftFront.SetInverted(false);
    leftBack.SetInverted(false);

    // Setup Gyro
    gyro.Calibrate();
    gyro.Reset();
    prevAngle = gyro.GetAngle();
}

//Called While Robot is on
void Robot::RobotPeriodic() {
    SmartDashboard::PutNumber("Arm Angle", arm.getAngle());
    SmartDashboard::PutNumber("Gyro Angle", gyro.GetAngle());
    SmartDashboard::PutNumber("Elevator Height (in)", elevator.getHeight());
}

//Called Initially on Autonomous Start
void Robot::AutonomousInit() {}

//Called During Autonomous
void Robot::AutonomousPeriodic() {}

//Called Initially on Teleop Start
void Robot::TeleopInit() {}

// Called During Teleop
void Robot::TeleopPeriodic() {
    handleDrivetrain();
    handleElevator();
    handleSpear();
    handleArm();
    handleCargoIntake();
}

// Copilot: Handles controller input use with flywheel
void Robot::handleCargoIntake() {
    if (copilot.RightY() > FLYWHEEL_THRESHOLD) {
        arm.setMode(Arm::OUTTAKE);
    } else if (copilot.RightY() < -FLYWHEEL_THRESHOLD) {
        arm.setMode(Arm::INTAKE);
    } else {
        arm.setMode(Arm::OFF);
    }
}

// Copilot: Handles controller input with pistons (Spear)
void Robot::handleSpear() {
    spear.setPlaceRoutine(copilot.ButtonState(GamepadF310::BUTTON_X));
    spear.setGrabRoutine(copilot.ButtonState(GamepadF310::BUTTON_Y));
    spear.updateRoutine();
}

// Pilot: Handles controller input for movement
void Robot::handleDrivetrain() {

    if (pilot.ButtonState(GamepadF310::BUTTON_LEFT_BUMPER)) {
    // Gearshifter
        gearState = LOW;
    } else if (pilot.ButtonState(GamepadF310::BUTTON_RIGHT_BUMPER)) {
        gearState = HIGH;
    }           
    
    gearShifter.Set(gearState);

    // Vision Autocorrect
    double turn = pilot.RightX();
    if (pilot.RightTrigger() > VISION_TRIGGER_THRESHOLD && SmartDashboard::GetBoolean("Target Acquired", false)) {
        int visionMid = SmartDashboard::GetNumber("Vision Mid X", CAMERA_WIDTH/2);
        int targetX = CAMERA_WIDTH/2 + SmartDashboard::GetNumber("Vision Target Pixel Width", 0)*TARGET_WIDTH_TO_CAMERA_OFFSET_RATIO;
        turn = sqrt(visionMid - targetX) / TURN_SCALE_FACTOR;
    }
    
    speed = Lib830::accel(prevSpeed, pilot.LeftY(), TICKS_TO_ACCEL);
    prevSpeed = speed;

    // Activates gyro correct on straight driving
    if (gyroCorrectState.toggle(pilot.ButtonState(GamepadF310::BUTTON_RIGHT_STICK))
        && (std::fabs(pilot.RightX()) < CONTROLLER_GYRO_THRESHOLD) && speed > SPEED_GYRO_THRESHOLD) {
        drivetrain.CurvatureDrive(speed, (prevAngle - gyro.GetAngle()) / (-90.0), std::fabs(speed)<DRIVETRAIN_DEADZONE_THRESHOLD);
    } else {
        drivetrain.CurvatureDrive(speed, turn, std::fabs(speed)<DRIVETRAIN_DEADZONE_THRESHOLD);
        prevAngle = gyro.GetAngle();
    }

    SmartDashboard::PutNumber("Drivetrain Turn", turn);
    SmartDashboard::PutBoolean("High Gear State", gearState);
}

// Copilot: Handles controller input with elevator
// manual lower: left trigger
// manual raise: right trigger
void Robot::handleElevator() {
    elevator.update();

    if (copilot.ButtonState(GamepadF310::BUTTON_LEFT_BUMPER) || copilot.ButtonState(GamepadF310::BUTTON_RIGHT_BUMPER)){
        elevatorMode = AUTOMATIC;
    }

    if (copilot.RightTrigger() > MANUAL_ELEVATOR_THRESHOLD || copilot.LeftTrigger() > MANUAL_ELEVATOR_THRESHOLD){
        elevatorMode = MANUAL;
    }


    leftBumper.toggle(copilot.ButtonState(GamepadF310::BUTTON_LEFT_BUMPER));
    rightBumper.toggle(copilot.ButtonState(GamepadF310::BUTTON_RIGHT_BUMPER));

    if (elevatorMode == MANUAL) {
        if (std::fabs(copilot.LeftTrigger()) > MANUAL_ELEVATOR_THRESHOLD) {
            elevator.setManualSpeed(-copilot.LeftTrigger());
        } else if (std::fabs(copilot.RightTrigger()) > MANUAL_ELEVATOR_THRESHOLD) {
            elevator.setManualSpeed(copilot.RightTrigger());
        } else {
            elevator.setManualSpeed(0);
        }
    } else if (leftBumper && !rightBumper) {
            elevator.changeSetpoint(-1);
    } else if (!leftBumper && rightBumper) {
        elevator.changeSetpoint(1);
    }

    leftBumper = false;
    rightBumper = false;

    SmartDashboard::PutBoolean("Manual Elevator", elevatorMode);
    SmartDashboard::PutString("Elevator Height", elevator.getSetpoint());
}


// Copilot: Handles controller input with rotating arm
// this becomes left stick 
void Robot::handleArm() {
    arm.update();
    armManualMode.toggle(copilot.ButtonState(GamepadF310::BUTTON_START));
    armUp.toggle(copilot.LeftY() > ARM_THRESHOLD);
    armDown.toggle(-copilot.LeftY() > ARM_THRESHOLD);
    
    double deadzoneLeftY = std::fabs(copilot.LeftY()) > ARM_THRESHOLD ? -copilot.LeftY() : 0;
    if (armManualMode) {
        arm.setManualSpeed(deadzoneLeftY);
    } else if (armDown) {
        arm.changeSetpoint(1);
    } else if (armUp) {
        arm.changeSetpoint(-1);
    } else {
        arm.setSetpoint();
    }
    armUp = false;
    armDown = false;
    SmartDashboard::PutBoolean("Manual Arm", armManualMode);
    SmartDashboard::PutString("Arm Height", arm.getSetpoint());
}

void Robot::TestPeriodic() {}
        //Called During Test

void Robot::DisabledInit(){
    spear.setExtend(false); 
    spear.setHatchGrab(false);
}

// Elevator::Elevator(WPI_TalonSRX &motor) : motor(motor) {
#ifndef RUNNING_FRC_TESTS
    int main() {
    frc::StartRobot<Robot>();
}
#endif
