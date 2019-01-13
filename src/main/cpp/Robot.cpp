/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <Robot.h>

void Robot::CameraLoop(){
    CameraServer &server = *CameraServer::GetInstance();
    GripPipeline pipeline;
  	cs::UsbCamera webcamfront {"Front Camera", 1};

    cv::Mat image;
    cv::Mat image_temp;
    cv::Mat hsl_output;
    int g_frame = 0;

    cs::CvSink sink;
    cs::CvSource outputStream;

    webcamfront = server.StartAutomaticCapture();
    webcamfront.SetResolution(320,240);

    sink = server.GetVideo();
    outputStream = server.PutVideo("Processed", 320, 240);

    while(1) {
        bool working = sink.GrabFrame(image_temp);
        SmartDashboard::PutBoolean("working", working);

        if (working) {
            g_frame++;
            image = image_temp;
        }
        if (g_frame < 1) {
            continue;
        }

        pipeline.Process(image);
        //outputStream.PutFrame(*pipeline.gethslThresholdOutput());
        outputStream.PutFrame(pipeline.resizeImageOutput);

    }
}

void Robot::RobotInit() {
    std::thread visionThread(CameraLoop);
    visionThread.detach();
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

    double speed = Lib830::accel(prev_speed, pilot.GetY(LEFTSIDE), TICKS_TO_ACCEL);
	prev_speed = speed;
    
    drivetrain.CurvatureDrive(speed, pilot.GetX(RIGHTSIDE), std::fabs(speed) < 0.05);

}
//Called During Teleop

void Robot::TestPeriodic() {}
//Called During Test

#ifndef RUNNING_FRC_TESTS
START_ROBOT_CLASS(Robot)
#endif
