#include <Lib830.h>
#include <ctre/Phoenix.h>
#include <frc/WPILib.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <vector>
class Arm {
   public:
    enum FlywheelMode { OFF, OUTTAKE, INTAKE };
    enum ArmMode { MANUAL, AUTO };
    Arm(WPI_VictorSPX &joint, WPI_VictorSPX &flywheel, frc::AnalogPotentiometer &pot);
    void setMode(FlywheelMode mode);
    void setAngle(int index);
    void setManualSpeed(double speed);
    double getAngle();
    int numSetpoints();


  private:
    WPI_VictorSPX &joint;
    WPI_VictorSPX &flywheel;
    frc::AnalogPotentiometer &pot;

    double p = 0.01;
    double i = 0.0;
    double d = 0.0;
    double f = 0.0;
    
    frc::PIDController armPID{p,i,d,f,pot,joint};
    double intake_height = 190.0;
    double spear_height = 140.0;
    double ball_height = 90.0;
    double inside_frame_perimeter = 35.0;
    std::vector<double> armAngles = {inside_frame_perimeter, ball_height, spear_height, intake_height};
    static constexpr double JOINT_ANGLE_THRESHOLD = 10.0;
    static constexpr double MAX_DOWN_SPEED = 0.25;
    static constexpr double MAX_UP_SPEED = 1.0;
};