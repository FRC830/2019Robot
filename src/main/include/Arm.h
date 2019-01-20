#include <Lib830.h>
#include <frc/WPILib.h>


// This class controls the movement of the angle & the intake or outtakes
class Arm {
    public:
      Arm(VictorSP &ArmMotor, VictorSP &FlywheelMotor, AnalogPotentiometer &ArmPotentiometer);
      void setIntake(Mode mode);
      void releasePistons();
      void setAngle(float angle);
      float getAngle();
    private:
      enum Mode { ON = true, OFF = false };
      static const float PI = 3.1415927;
      static const int ENCODER_TICKS = 1024;
      static const int WINCH_DIAMETER = 6; // PLACEHOOLDER;
      static const int ENCODER_TICK_DISTANCE = 6 * PI / ENCODER_TICKS;

      VictorSP arm{WINCH_MOTOR_PORT};
      VictorSP flywheel{FLYWHEEL_MOTOR_PORT};
      AnalogPotentiometer armPot{POTENTIOMETER_ANALOG_PORT};
};

