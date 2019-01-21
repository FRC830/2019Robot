#include <Lib830.h>
#include <frc/WPILib.h>


// This class controls the movement of the angle & the intake or outtakes
class Arm {
    public:
      enum Mode
      {
        OUTTAKE,
        INTAKE
      };
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
};

