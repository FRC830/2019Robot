/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <Robot.h>

using namespace frc;
using namespace Lib830;
// BAG Motor with encoders
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
    webcamFront.SetResolution(320,240);
    webcamFront.SetExposureManual(20);
    webcamFront.SetFPS(30);

    sink = server.GetVideo();
    outputStream = server.PutVideo("Processed", 320, 240);

    while(1) {
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

void Robot::RobotInit() {
    //Setup camera tools
    std::thread visionThread(CameraLoop);
    visionThread.detach();

    //Start gyro
    gyro.Calibrate();
    gyro.Reset();
    prevAngle = gyro.GetAngle();
    elevatorEncoder.SetDistancePerPulse(ENCODER_TICK_DISTANCE);
}

void Robot::RobotPeriodic() {
    frc::SmartDashboard::PutNumber("Height: ", heights[currentHeight]);
}
//Called Whilst Robot is on

void Robot::AutonomousInit() {}
//Called Initially on Autonomous Start

void Robot::AutonomousPeriodic() {}
//Called During Autonomous

void Robot::TeleopInit() {}
//Called Initially on Teleop Start

void Robot::TeleopPeriodic() {
    handleDrivetrain();
    handleElevator();
    handlePistons();
    handleJoint();
    handleFlywheel();

}
void Robot::handleFlywheel() {
    if (copilot.LeftY() > FLYWHEEL_THRESHOLD)
    {
        arm.setMode(Arm::OUTTAKE);
    }
    else if (copilot.LeftY() < -(FLYWHEEL_THRESHOLD))
    {
        arm.setMode(Arm::INTAKE);
    }
}
void Robot::handlePistons() {
    if (copilot.ButtonState(GamepadF310::BUTTON_A)) {
        arm.releasePistons();
    }
    arm.update();
}
void Robot::handleDrivetrain() {
    speed = Lib830::accel(prevSpeed, pilot.LeftY(), TICKS_TO_ACCEL);
    prevSpeed = speed;
    speed = Lib830::accel(prevSpeed, pilot.LeftY(), TICKS_TO_ACCEL);
    prevSpeed = speed;
    // Activates gyro correct on straight driving
    double controller_threshold = 0.05;
    if (std::fabs(pilot.RightX()) > controller_threshold) {
        drivetrain.CurvatureDrive(speed, pilot.RightX(), std::fabs(speed) < controller_threshold);
        prevAngle = gyro.GetAngle();
    } else {
        drivetrain.CurvatureDrive(speed, (prevAngle - gyro.GetAngle()) / (-90.0), std::fabs(speed) < controller_threshold);
    }
}
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
void Robot::handleJoint() {
    if (copilot.DPadY() != 0) {
        arm.setAngle(arm.getAngle() + copilot.DPadY());
    }
}
void Robot::TestPeriodic() {}
        //Called During Test

#ifndef RUNNING_FRC_TESTS
int main(){
    frc::StartRobot<Robot>();
}
#endif