#include <Lib830.h>
#include <frc/WPILib.h>
#include <ctre/Phoenix.h>
#include <vector>

class Arm {
    public:
      enum FlywheelMode { OFF, OUTTAKE, INTAKE };
      enum ArmMode { MANUAL, AUTO };
      Arm(WPI_VictorSPX &joint, WPI_VictorSPX &flywheel, frc::AnalogPotentiometer &pot);
      void setMode(FlywheelMode mode);
      void setAngle(int index);
      void setManualSpeed(double speed);
      double getAngle();
    private:
      WPI_VictorSPX &joint;
      WPI_VictorSPX &flywheel;
      frc::AnalogPotentiometer &pot;
      frc::Timer timer;
      static const int LOW = 120;
      static const int SPEAR_HEIGHT = 90;
      static const int HIGH = 30;
      static const int INSIDE_FRAME_PERIMETER = 0;
      std::vector<int> armHeights = { INSIDE_FRAME_PERIMETER, HIGH, SPEAR_HEIGHT, LOW };
      static constexpr double JOINT_ANGLE_THRESHOLD = 5.0;
      static constexpr double PROPORTION = 1/300.0;
};

