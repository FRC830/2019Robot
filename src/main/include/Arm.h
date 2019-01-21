#include <Lib830.h>
#include <frc/WPILib.h>

class Arm {
    public:
      enum Mode { OUTTAKE, INTAKE };
      Arm(frc::VictorSP &joint, frc::VictorSP &flywheel, frc::AnalogPotentiometer &pot, frc::Solenoid &piston);
      void setMode(Mode mode);
      void releasePistons();
      void setAngle(double angle);
      double getAngle();
      void update();
    private:
      frc::VictorSP &joint;
      frc::VictorSP &flywheel;
      frc::AnalogPotentiometer &pot;
      frc::Solenoid &piston;
      frc::Timer timer;
      static constexpr double PISTON_EXTENSION_TIME = 0.5;
      static constexpr double JOINT_ANGLE_THRESHOLD = 5.0;
};

