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
    rightFront.Follow(rightBack);
    leftFront.Follow(leftBack);

    // For if we need to change it later
    rightBack.SetSensorPhase(false);
    leftBack.SetSensorPhase(false);
    rightFront.SetInverted(false);
    rightBack.SetInverted(false);
    leftFront.SetInverted(false);
    leftBack.SetInverted(false);

    // Setup Gyro
    gyro.Calibrate();
    gyro.Reset();
    prevAngle = gyro.GetAngle();
}

//Called Whilst Robot is on
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

    spear.setPlaceRoutine(copilot.LeftTrigger() > SPEAR_TRIGGER_THRESHOLD);
    spear.setGrabRoutine(copilot.RightTrigger() > SPEAR_TRIGGER_THRESHOLD);
    spear.updateRoutine();
}

double Robot::drivetrainDeadzone(double d) {
    if (std::fabs(d) < DRIVETRAIN_DEADZONE_THRESHOLD) {
        return 0;
    }
    return d;
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
        && (std::fabs(pilot.RightX()) < CONTROLLER_GYRO_THRESHOLD)) {
        drivetrain.CurvatureDrive(speed, (prevAngle - gyro.GetAngle()) / (-90.0), drivetrainDeadzone(speed));
    } else {
        drivetrain.CurvatureDrive(speed, turn, drivetrainDeadzone(speed));
        prevAngle = gyro.GetAngle();
    }
    
    SmartDashboard::PutNumber("Drivetrain Turn", turn);
    SmartDashboard::PutNumber("Gyro Angle", gyro.GetAngle());
    SmartDashboard::PutBoolean("Gear State", gearState);
}

// Copilot: Handles controller input with elevator
void Robot::handleElevator() {

    leftBumper.toggle(copilot.ButtonState(GamepadF310::BUTTON_LEFT_BUMPER));
    rightBumper.toggle(copilot.ButtonState(GamepadF310::BUTTON_RIGHT_BUMPER));

    if (std::fabs(copilot.LeftY()) > MANUAL_ELEVATOR_THRESHOLD) {
        elevator.setManualSpeed(copilot.LeftY());
    } else if ((leftBumper && !rightBumper) && currentSetpoint > 0) {
        currentSetpoint--;
        elevator.setSetpoint(currentSetpoint);
    }
    else if (leftBumper && !rightBumper && currentSetpoint < (elevator.numSetpoints() - 1)) {
        currentSetpoint++;
        elevator.setSetpoint(currentSetpoint);
    } else {
        elevator.setManualSpeed(0);
    }
    leftBumper = false;
    rightBumper = false;

    SmartDashboard::PutNumber("Setpoint", currentSetpoint);
    SmartDashboard::PutNumber("Height (in)", elevator.getHeight());
}


// Copilot: Handles controller input with rotating arm
void Robot::handleArm() {
    arm.update();    
    if (copilot.DPadY() != 0) {
        arm.setAngle(arm.getAngle() + copilot.DPadY() * JOINT_MOVEMENT_SPEED);
    }
    SmartDashboard::PutNumber("Arm Angle", arm.getAngle());
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