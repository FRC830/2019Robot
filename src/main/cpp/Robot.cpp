/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <Robot.h>

using namespace frc;
using namespace Lib830;

// Called Initially on Robot Start
void Robot::RobotInit() {
    // Reset Motors
    rightFront.ConfigFactoryDefault();
    rightBack.ConfigFactoryDefault();
    leftFront.ConfigFactoryDefault();
    leftBack.ConfigFactoryDefault();

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
    handleVision();
    handleDrivetrain();
    handleElevator();
    handleSpear();
    handleArm();
    handleFlywheel();
}

double combineIndividualPrevAndCurrentData(double prev, double current) {
    if (prev == -1) {
        return current;
    }
    if (current == -1) {
        return prev;
    }

    return (prev + current) / 2;
}

std::vector<double> Robot::combinePrevAndCurrentVisionData() {
    std::vector<double> result;
    result.push_back(combineIndividualPrevAndCurrentData(prevLeftRectArea, currentLeftRectArea));
    result.push_back(combineIndividualPrevAndCurrentData(prevRightRectArea, currentRightRectArea));
    result.push_back(combineIndividualPrevAndCurrentData(prevTargetMidpoint, currentTargetMidpoint));

    return result;
}

//The vision code for if we have the midpoint and both rectangle areas
void Robot::doFullDataVision(std::vector<double> data) {
    doMidpointOnlyVision(data[2]); //Maybe do something special here later
    /*double ratio = data[0] / data[1];
    double inv = 1.0 / ratio;
    double maxOfInverses = ratio > inv? ratio: inv;

    double difference = maxOfInverses - 1;
    double direction = ratio*/
}

//The vision code for if we only have the midpoint
void Robot::doMidpointOnlyVision(double midpoint) {
    Robot::visionSteer = (midpoint - 160) / 160;
}

void Robot::handleVision() {
    doingAutoAlign = pilot.ButtonState(GamepadF310::BUTTON_B);

    if (doingAutoAlign) {
        Robot::currentLeftRectArea = SmartDashboard::GetNumber("Left Rect Area", -1);
        Robot::currentRightRectArea = SmartDashboard::GetNumber("Right Rect Area", -1);
        Robot::currentTargetMidpoint = SmartDashboard::GetNumber("Vision Mid X", -1);

        std::vector<double> combined = combinePrevAndCurrentVisionData();

        
        if(combined[0] != -1 && combined[1] != -1 && combined[2] != -1) {
            doFullDataVision(combined);
        } else if (combined[2] != 0) {
            doMidpointOnlyVision(combined[2]);
        } else {
            visionSteer = false; //We don't have data, so we can't steer
        }
    }
}

// Copilot: Handles controller input use with flywheel
void Robot::handleFlywheel() {
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
    static Toggle spearExtendTog;
    // Manual setting of spear
    // spear.setExtend(spearExtendTog.toggle(copilot.ButtonState(GamepadF310::BUTTON_A)));
    // spear.setHatchGrab(copilot.RightTrigger() > 0.25);
    // Routines are a full set of instructions
    spear.setPlaceRoutine(copilot.ButtonState(GamepadF310::BUTTON_X));
    spear.setGrabRoutine(copilot.ButtonState(GamepadF310::BUTTON_Y));
    spear.updateRoutine();
}

double deadzone(double d) {
 if (std::fabs(d) < 0.05) {
     return 0;
    } else {
    return d;
    }

}
// Pilot: Handles controller input for movement
void Robot::handleDrivetrain() {

    speed = Lib830::accel(prevSpeed, pilot.LeftY(), TICKS_TO_ACCEL);
    prevSpeed = speed;

    // Activates gyro correct on straight driving
    if(visionSteer) {
        drivetrain.CurvatureDrive(speed, visionSteer, false);
    }
    else {
        if ((std::fabs(pilot.RightX()) < CONTROLLER_GYRO_THRESHOLD) && gyroCorrectEnabled) {
            drivetrain.CurvatureDrive(speed, (prevAngle - gyro.GetAngle()) / (-90.0), std::fabs(speed) < 0.05);
        } else {
            // cout<<"you've got mail;";
            drivetrain.CurvatureDrive(speed, pilot.RightX(), std::fabs(speed) < 0.05);
            prevAngle = gyro.GetAngle();
        }
    }
}

// Copilot: Handles controller input with elevator
// TODO FIX THIS
void Robot::handleElevator() {
    if (copilot.ButtonState(GamepadF310::BUTTON_LEFT_BUMPER) &&
        !copilot.ButtonState(GamepadF310::BUTTON_RIGHT_BUMPER) && currentHeight != heights.size() - 1) {
        if (!bumperPressed) {
            elevator.setHeight(heights[currentHeight -= 1]);
        }
        bumperPressed = true;
    } else if (copilot.ButtonState(GamepadF310::BUTTON_RIGHT_BUMPER) &&
             !copilot.ButtonState(GamepadF310::BUTTON_LEFT_BUMPER) && currentHeight != 0) {
        if (!bumperPressed) {
            elevator.setHeight(heights[currentHeight += 1]);
        }
        bumperPressed = true;
    } else {
        bumperPressed = false;
    }
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