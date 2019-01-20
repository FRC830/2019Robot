#include <Lib830.h>
#include <frc/WPILib.h>


// This class controls the movement of the angle & the intake or outtakes
class Arm {
    public:
      Arm(frc::VictorSP &ArmMotor, frc::VictorSP &FlywheelMotor, frc::AnalogPotentiometer &ArmPotentiometer);
      void setIntake(bool mode);
      void releasePistons();
      void setAngle(double angle);
      double getAngle();

    private:
      frc::VictorSP &arm;
      frc::VictorSP &flywheel;
      frc::AnalogPotentiometer &armPot;
};

