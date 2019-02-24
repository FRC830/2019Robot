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
    void update();


  private:
    WPI_VictorSPX &joint;
    WPI_VictorSPX &flywheel;
    frc::AnalogPotentiometer &pot;

    double p = 0.03, i = 0.0, d = 0.0, f = 0.0;
    nt::NetworkTableEntry nt_p, nt_i, nt_d, nt_f, nt_ball_intake, nt_inside_frame_perimeter, nt_ball_outtake, nt_spear_angle;
    frc::PIDController armPID{p,i,d,f,pot,joint};
    double ball_intake = 183.0;
    double spear_angle = 140.0;
    double ball_outtake = 90.0;
    double inside_frame_perimeter = 35.0;
    std::vector<double> armAngles = {inside_frame_perimeter, ball_outtake, spear_angle, ball_intake};
    static constexpr double JOINT_ANGLE_THRESHOLD = 10.0;
    static constexpr double MAX_DOWN_SPEED = 0.25;
    static constexpr double MAX_UP_SPEED = 1.0;
};