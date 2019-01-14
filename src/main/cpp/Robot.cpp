/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <Robot.h>

using namespace frc;

void Robot::CameraLoop(){
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
        frc::SmartDashboard::PutBoolean("working", working);

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
    std::thread visionThread(CameraLoop);
    visionThread.detach();
    gyro.Reset();
}

void Robot::RobotPeriodic() {}
//Called Whilst Robot is on

void Robot::AutonomousInit() {}
//Called Initially on Autonomous Start

void Robot::AutonomousPeriodic() {}
//Called During Autonomous

void Robot::TeleopInit() {}
//Called Initially on Teleop Start

void Robot::TeleopPeriodic() {
    frc::SmartDashboard::PutNumber("Gyro",gyro.GetAngle());
}
//Called During Teleop

void Robot::TestPeriodic() {}
//Called During Test

#ifndef RUNNING_FRC_TESTS
int main(){
    frc::StartRobot<Robot>();
}
#endif
