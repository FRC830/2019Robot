#include <Lib830.h>
#include <frc/WPILib.h>
#include <ctre/Phoenix.h>


class Spear {
    public:
      Spear(frc::Solenoid &hatchGrabPiston, frc::Solenoid &extensionPiston);
      void setExtend (bool);
      void setHatchGrab (bool);
    private:
      frc::Solenoid &hatchGrabPiston;
      frc::Solenoid &extensionPiston;
      static constexpr double HATCH_GRAB_PISTON_EXTENSION_TIME = 0.5;
      static constexpr double EXTENSION_PISTON_EXTENSION_TIME = 0.5;
};

