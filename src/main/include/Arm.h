#include <Lib830.h>
#include <ctre/Phoenix.h>
#include <frc/WPILib.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <vector>
class Arm {
   public:
    enum FlywheelMode { OFF = 0 , OUTTAKE = 1, INTAKE = -1};
    enum ArmMode { MANUAL, AUTO };
    Arm(WPI_VictorSPX &joint, WPI_VictorSPX &flywheel, frc::AnalogPotentiometer &pot);
    void setMode(FlywheelMode mode);
    void changeSetpoint(int change);
    void setManualSpeed(double speed);
    double getAngle();
    std::string getSetpoint();
    void setSetpoint();
    void update();


  private:
    WPI_VictorSPX &joint;
    WPI_VictorSPX &flywheel;
    frc::AnalogPotentiometer &pot;

    double p = 0.03, i = 0.0, d = 0.0, f = 0.0;
    nt::NetworkTableEntry nt_p, nt_i, nt_d, nt_f, nt_ball_intake, nt_spear_intake, nt_inside_frame_perimeter, nt_ball_outtake, nt_spear_angle;
    double ball_intake = 183.0, spear_intake = 161.0, spear_angle = 140.0, ball_outtake = 90.0, inside_frame_perimeter = 35.0;

    frc::PIDController armPID{p,i,d,f,pot,joint};

    std::vector<nt::NetworkTableEntry> ntAngles = {nt_inside_frame_perimeter, nt_ball_outtake, nt_spear_angle, nt_spear_intake};
    std::vector<double> defaultAngles = {inside_frame_perimeter, ball_outtake, spear_angle, spear_angle};
    std::vector<double> angles = defaultAngles; // shouldn't be a pointer b/c vector
    std::vector<std::string> armHeightWords = {"INSIDE FRAME", "OUTTAKE BALL", "SPEAR PLACE HEIGHT", "BALL INTAKE"};
    int currentSetpoint = 0;

    static constexpr double JOINT_ANGLE_THRESHOLD = 15.0;
    static constexpr double MAX_DOWN_SPEED = 1.0;
    static constexpr double MAX_UP_SPEED = 1.0;
};