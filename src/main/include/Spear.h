#include <Lib830.h>
#include <frc/WPILib.h>
#include <ctre/Phoenix.h>



class Spear {
    public:
      Spear(frc::Solenoid &hatchGrabPiston, frc::Solenoid &extensionPiston);

      void setPlaceRoutine(bool running);
      void setGrabRoutine(bool grabbing);
      void updateRoutine();
      void setExtend(bool state);
      void setHatchGrab(bool state);
    private:
      frc::Timer timer;
      frc::Solenoid &hatchGrabPiston;
      frc::Solenoid &extensionPiston;

      // Values set the time in-between actions
      static constexpr double FLIP_DELAY = 0.25;

      bool placing = false;
      bool grabbing = false;
      bool end_placing = false;
      bool end_grabbing = false;
      bool previous_placing = false;
      bool previous_grabbing = false;

};

