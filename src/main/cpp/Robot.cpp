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
    } else if (copilot.RightY() < -(FLYWHEEL_THRESHOLD)) {
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

    // Gearshifter
    if (pilot.ButtonState(GamepadF310::BUTTON_LEFT_BUMPER)) {
        gearState = LOW;
    } else if (pilot.ButtonState(GamepadF310::BUTTON_RIGHT_BUMPER)) {
        gearState = HIGH;
    }           
    
    gearShifter.Set(gearState);

    // Vision Autocorrect
    double turn = pilot.RightX();
    if (pilot.RightTrigger() > VISION_TRIGGER_THRESHOLD && SmartDashboard::GetBoolean("Target Acquired", false)) {
        int visionMid = SmartDashboard::GetNumber("Vision Mid X", CAMERA_WIDTH/2);
        turn = sqrt(visionMid - CAMERA_WIDTH / 2) / TURN_SCALE_FACTOR;
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
    SmartDashboard::PutNumber("Gyro Angle", gyro.GetAngle());
    SmartDashboard::PutBoolean("Gear State", gearState);
}

// Copilot: Handles controller input with elevator
// manual lower: left trigger
// manual raise: right trigger
void Robot::handleElevator() {

    if (copilot.ButtonState(GamepadF310::BUTTON_LEFT_BUMPER) || copilot.ButtonState(GamepadF310::BUTTON_RIGHT_BUMPER)){
        elevatorMode = AUTOMATIC;
    }

    //deadzone
    if (copilot.RightTrigger() > MANUAL_ELEVATOR_THRESHOLD || copilot.LeftTrigger() > MANUAL_ELEVATOR_THRESHOLD){
        elevatorMode = MANUAL;
    }


    leftBumper.toggle(copilot.ButtonState(GamepadF310::BUTTON_LEFT_BUMPER));
    rightBumper.toggle(copilot.ButtonState(GamepadF310::BUTTON_RIGHT_BUMPER));

    if (elevatorMode == MANUAL){
        // manual lower
        if (std::fabs(copilot.LeftTrigger()) > MANUAL_ELEVATOR_THRESHOLD) {
            elevator.setManualSpeed(-(copilot.LeftTrigger()));
        } else if (std::fabs(copilot.RightTrigger()) > MANUAL_ELEVATOR_THRESHOLD) {
            elevator.setManualSpeed(copilot.RightTrigger());
        } else {
            elevator.setManualSpeed(0);
        }
    } else {
        //automatic setpoints, right bumper is up one, left bumper is down one
        if ((leftBumper && !rightBumper) && currentSetpoint > 0) {
            currentSetpoint--;
            elevator.setSetpoint(currentSetpoint);
        }
        else if (leftBumper && !rightBumper && currentSetpoint < (elevator.numSetpoints() - 1)) {
            currentSetpoint++;
            elevator.setSetpoint(currentSetpoint);
        }
    }

    leftBumper = false;
    rightBumper = false;

    SmartDashboard::PutBoolean("Manual Elevator", elevatorMode);
    SmartDashboard::PutNumber("Setpoint", currentSetpoint);
    SmartDashboard::PutNumber("Height (in)", elevator.getHeight());
}


// Copilot: Handles controller input with rotating arm
// this becomes left stick 
void Robot::handleArm() {
    armMode.toggle(copilot.ButtonState(GamepadF310::BUTTON_START));
    armUp.toggle(copilot.LeftY() > ARM_THRESHOLD);
    armDown.toggle(-(copilot.LeftY()) > ARM_THRESHOLD);
    
    double deadzoneLeftY = (std::fabs(copilot.LeftY()) > ARM_THRESHOLD ? copilot.LeftY() : 0);
    if (armMode) {
        arm.setManualSpeed(deadzoneLeftY);
    } else if (armDown && currentArmSetpoint < (arm.numSetpoints() - 1)) {
        currentSetpoint++;
    } else if (armUp && currentArmSetpoint > 0) {
        currentArmSetpoint--;
    } else {
        arm.setAngle(currentArmSetpoint);
    }
    armUp = false;
    armDown = false;
    SmartDashboard::PutNumber("Arm Angle", arm.getAngle());
    SmartDashboard::PutBoolean("Manual Arm", armMode);
}

void Robot::TestPeriodic() {}
        //Called During Test

void Robot::DisabledInit(){
    spear.setExtend(false); 
    spear.setHatchGrab(false);
}

#ifndef RUNNING_FRC_TESTS
int main(){
    frc::StartRobot<Robot>();
}
#endif
