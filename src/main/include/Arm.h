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

    double p = 1/100.0;
    double i = 0.0;
    double d = 0.0;
    double f = 0;
    
    frc::PIDController armPID{p,i,d,f,pot,joint};
    static const int LOW = 190;
    static const int SPEAR_HEIGHT = 130;
    static const int HIGH = 100;
    static const int INSIDE_FRAME_PERIMETER = 40;
    std::vector<int> armHeights = {INSIDE_FRAME_PERIMETER, HIGH, SPEAR_HEIGHT, LOW};
    static constexpr double JOINT_ANGLE_THRESHOLD = 5.0;
};