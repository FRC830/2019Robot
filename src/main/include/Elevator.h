#include <Lib830.h>
#include <ctre/Phoenix.h>
#include <frc/WPILib.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/BuiltInWidgets.h>
class Elevator {
    public:
        Elevator(WPI_TalonSRX &motor);
        double getHeight();
        void changeSetpoint(int change);
        void setManualSpeed(double speed);
        std::string getSetpoint();
        void update();
    private:
        WPI_TalonSRX &motor;
        static const int ENCODER_TICKS = 4096;
        static constexpr double PI = 3.1415927;
        static constexpr double MOTOR_DIAMETER = 4.0/7.0;
        static constexpr double ENCODER_TICK_DISTANCE = MOTOR_DIAMETER * PI / ENCODER_TICKS;

        // Distance to offset flywheel/spear against center of target
        static constexpr double FLYWHEEL_OFFSET = 0;
        static constexpr double SPEAR_OFFSET = 0;
        static constexpr double DIVIDER_DISTANCE = 28;

        // fhh -> (first) (hatch) (height)
        nt::NetworkTableEntry nt_fhh, nt_shh, nt_thh, nt_fbh, nt_sbh, nt_tbh, nt_max_down;

        // Heights of all targets, In inches Defaults
        double first_hatch = 19 + SPEAR_OFFSET, first_ball = 27.5 + FLYWHEEL_OFFSET;

        int currentSetpoint = 0;
        // Motor Configuration Defaults
        double p = 0.02, i = 0, d = 0, f = 0.0;
        double max_down_speed = 1.0;
        bool motorFlipped = false;
        bool encoderFlipped = false;

        // value arrays
        nt::NetworkTableEntry nt_p, nt_i, nt_d, nt_f, nt_motorFlipped, nt_encoderFlipped;
        std::vector<nt::NetworkTableEntry> ntHeights = {nt_fhh, nt_fbh, nt_shh, nt_sbh, nt_thh, nt_tbh};
        std::vector<double> defaultHeights = { first_hatch, first_ball, first_hatch + DIVIDER_DISTANCE, first_ball + DIVIDER_DISTANCE, first_hatch + DIVIDER_DISTANCE * 2, first_ball + DIVIDER_DISTANCE * 2};
        std::vector<double> heights = defaultHeights;
        std::vector<std::string> elevatorHeightWords = {"BOTTOM HATCH","BOTTOM BALL","MID HATCH","MID BALL","TOP HATCH","TOP BALL"};
};