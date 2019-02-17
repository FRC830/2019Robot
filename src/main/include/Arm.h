#include <Lib830.h>
#include <frc/WPILib.h>
#include <ctre/Phoenix.h>
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
    double f = 0;
    
    frc::PIDController armPID{p,i,d,f,pot,joint};
    static const int INTAKE_HEIGHT = 190;
    static const int SPEAR_HEIGHT = 140;
    static const int BALL_HEIGHT = 90;
    static const int INSIDE_FRAME_PERIMETER = 35;
    std::vector<int> armHeights = {INSIDE_FRAME_PERIMETER, BALL_HEIGHT, SPEAR_HEIGHT, INTAKE_HEIGHT};
    static constexpr double JOINT_ANGLE_THRESHOLD = 10.0;
    static constexpr double MAX_DOWN_SPEED = 0.25;
    static constexpr double MAX_UP_SPEED = 1.0;
};