#include <Lib830.h>
#include <frc/WPILib.h>
#include <ctre/Phoenix.h>


class Spear {
    public:
      Spear(frc::Solenoid &hatchGrabPiston, frc::Solenoid &extensionPiston);
      void setExtend (bool state);
      void setHatchGrab (bool state);
      void setPlaceRoutine(bool running);
      void setGrabRoutine(bool running);
      void updateRoutine();
    private:
      frc::Timer timer;
      frc::Solenoid &hatchGrabPiston;
      frc::Solenoid &extensionPiston;
      // Values set the time in-between actions
      static constexpr double PLACING_EXTEND_TIME = 0.25;
      static constexpr double PLACING_FLIP_DOWN_TIME = 0.15;
      static constexpr double PLACING_RETRACT_TIME = 0.25;
      
      static constexpr double GRABBING_FLIP_DOWN_TIME = 0.15;
      static constexpr double GRABBING_EXTEND_TIME = 0.25;
      static constexpr double GRABBING_FLIP_UP_TIME = 0.15;
      bool placing = false;
      bool grabbing = false;
};

