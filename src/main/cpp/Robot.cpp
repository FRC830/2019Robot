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

    // Set Motor Direction (Currently does nothing, may be useful for hot swapping)
    rightFront.SetInverted(false);
    rightBack.SetInverted(false);
    leftFront.SetInverted(false);
    leftBack.SetInverted(false);

    // Setup Gyro
    gyro.Calibrate();
    gyro.Reset();
    prevAngle = gyro.GetAngle();

    // Setup Encoder
    elevatorEncoder.SetDistancePerPulse(ENCODER_TICK_DISTANCE);
}

//Called Whilst Robot is on
void Robot::RobotPeriodic() {
    frc::SmartDashboard::PutNumber("Height: ", heights[currentHeight]);
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
    if (copilot.LeftY() > FLYWHEEL_THRESHOLD) {
        arm.setMode(Arm::OUTTAKE);
    } else if (copilot.LeftY() < -(FLYWHEEL_THRESHOLD)) {
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

double Robot::deadzone(double d) {
    if (std::fabs(d) < CONTROLLER_DEADZONE_THRESHOLD) {
        return 0;
    } else {
        return d;
    }
}
// Pilot: Handles controller input for movement
void Robot::handleDrivetrain() {
    // Gearshifter
    if (pilot.ButtonState(GamepadF310::BUTTON_LEFT_BUMPER)){
        gearState = LOW;
    }
    if(pilot.ButtonState(GamepadF310::BUTTON_RIGHT_BUMPER)){
        gearState = HIGH;
    }           
    
    gearShifter.Set(gearState);


    double turn = pilot.RightX();
    if (pilot.RightTrigger() > 0.3 && SmartDashboard::GetBoolean("Target Acquired", false)) {
        int visionMid = SmartDashboard::GetNumber("Vision Mid X", 160);
        turn = (visionMid-160)/580.0;
    }
    
    speed = Lib830::accel(prevSpeed, pilot.LeftY(), TICKS_TO_ACCEL);
    prevSpeed = speed;

    // Activates gyro correct on straight driving
    if (gyroCorrectState.toggle(pilot.ButtonState(GamepadF310::BUTTON_RIGHT_STICK))
        && (std::fabs(pilot.RightX()) < CONTROLLER_GYRO_THRESHOLD)) {
        drivetrain.CurvatureDrive(speed, (prevAngle - gyro.GetAngle()) / (-90.0), deadzone(speed));
    } else {
        drivetrain.CurvatureDrive(speed, turn, deadzone(speed));
        prevAngle = gyro.GetAngle();
    }
}

// Copilot: Handles controller input with elevator
void Robot::handleElevator() {

    leftBumper.toggle(copilot.ButtonState(GamepadF310::BUTTON_LEFT_BUMPER));
    rightBumper.toggle(copilot.ButtonState(GamepadF310::BUTTON_RIGHT_BUMPER));
    
    if ((leftBumper && !rightBumper) && currentHeight != heights.size() - 1) {
        elevator.setHeight(heights[currentHeight -= 1]);
    } else if (leftBumper && !rightBumper && currentHeight != 0) {
        elevator.setHeight(heights[currentHeight += 1]);
    }

    leftBumper = false;
    rightBumper = false;
}

// Copilot: Handles controller input with rotating arm
void Robot::handleArm() {
    arm.update();    
    if (copilot.DPadY() != 0) {
        arm.setAngle(arm.getAngle() + copilot.DPadY() * JOINT_MOVEMENT_SPEED);
    }
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