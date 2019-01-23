#include <Lib830.h>
#include <frc/WPILib.h>
#include <ctre/Phoenix.h>


class Arm {
    public:
      enum Mode { OFF, OUTTAKE, INTAKE };
      Arm(WPI_VictorSPX &joint, WPI_VictorSPX &flywheel, frc::AnalogPotentiometer &pot, frc::Solenoid &puncher);
      void setMode(Mode mode);
      void releasePistons();
      void setAngle(double angle);
      double getAngle();
      void update();
    private:
      WPI_VictorSPX &joint;
      WPI_VictorSPX &flywheel;
      frc::AnalogPotentiometer &pot;
      frc::Solenoid &puncher;
      frc::Timer timer;
      static constexpr double PISTON_EXTENSION_TIME = 0.5;
      static constexpr double JOINT_ANGLE_THRESHOLD = 5.0;
};

