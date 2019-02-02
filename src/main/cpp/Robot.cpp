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

    // Setup camera tools
    std::thread visionThread(CameraLoop);
    visionThread.detach();

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
    handleFlywheel();

}

// Seperate Thread For Camera Processing
void Robot::CameraLoop() {
    CameraServer &server = *CameraServer::GetInstance();
    GripPipeline pipeline;
    // cs::UsbCamera webcamfront {"Front Camera", 1};

    cv::Mat image;
    cv::Mat image_temp;
    cv::Mat hsl_output;
    int g_frame = 0;

    cs::CvSink sink;
    cs::CvSource outputStream;

    cs::UsbCamera webcamFront = server.StartAutomaticCapture();
    webcamFront.SetResolution(320, 240);
    webcamFront.SetExposureManual(20);
    webcamFront.SetFPS(30);

    sink = server.GetVideo();
    outputStream = server.PutVideo("Processed", 320, 240);

    while (1) {
        bool working = sink.GrabFrame(image_temp);

        if (working) {
            g_frame++;
            image = image_temp;
        }
        if (g_frame < 1) {
            continue;
        }

        pipeline.Process(image);
        // outputStream.PutFrame(*pipeline.gethslThresholdOutput());
        outputStream.PutFrame(image);
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
    spear.setExtend(spearExtendTog.toggle(copilot.ButtonState(GamepadF310::BUTTON_A)));
    spear.setHatchGrab(copilot.RightTrigger() > 0.25);
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
    if ((std::fabs(pilot.RightX()) < CONTROLLER_GYRO_THRESHOLD) && gyroCorrectEnabled) {
        drivetrain.CurvatureDrive(speed, (prevAngle - gyro.GetAngle()) / (-90.0), std::fabs(speed) < 0.05);
    } else {
        // cout<<"you've got mail;";
        drivetrain.CurvatureDrive(speed, pilot.RightX(), std::fabs(speed) < 0.05);
        prevAngle = gyro.GetAngle();
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