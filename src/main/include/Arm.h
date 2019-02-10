#include <Lib830.h>
#include <frc/WPILib.h>
#include <ctre/Phoenix.h>


class Arm {
    public:
      enum Mode { OFF, OUTTAKE, INTAKE };
      enum Setpoint { LOW = 120, SPEAR_HEIGHT = 90, HIGH = 30, INSIDE_FRAME_PERIMETER = 0};
      Arm(WPI_VictorSPX &joint, WPI_VictorSPX &flywheel, frc::AnalogPotentiometer &pot);
      void setMode(Mode mode);
      void setAngle(double angle);
      void setManualSpeed(double speed);
      double getAngle();
    private:
      WPI_VictorSPX &joint;
      WPI_VictorSPX &flywheel;
      frc::AnalogPotentiometer &pot;
      frc::Timer timer;
      Setpoint curSetpoint = INSIDE_FRAME_PERIMETER;
      static constexpr double JOINT_ANGLE_THRESHOLD = 5.0;
      static constexpr double PROPORTION = 1/300.0;
};

