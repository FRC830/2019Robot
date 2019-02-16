#include <Lib830.h>
#include <frc/WPILib.h>
#include <ctre/Phoenix.h>

class Elevator {
    public:
        Elevator(WPI_TalonSRX &motor);
        double getHeight();
        void setSetpoint(int height);
        void setManualSpeed(double speed);
        int numSetpoints();
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

        // Heights of all targets, In inches
        static constexpr double FIRST_HATCH_HEIGHT = 19 + SPEAR_OFFSET;
        static constexpr double SECOND_HATCH_HEIGHT = FIRST_HATCH_HEIGHT + DIVIDER_DISTANCE;
        static constexpr double THIRD_HATCH_HEIGHT = SECOND_HATCH_HEIGHT + DIVIDER_DISTANCE;

        static constexpr double FIRST_BALL_HEIGHT = 27.5 + FLYWHEEL_OFFSET;
        static constexpr double SECOND_BALL_HEIGHT = FIRST_BALL_HEIGHT + DIVIDER_DISTANCE;
        static constexpr double THIRD_BALL_HEIGHT = SECOND_BALL_HEIGHT + DIVIDER_DISTANCE;

        std::vector<double> heights = {
            FIRST_HATCH_HEIGHT, 
            FIRST_BALL_HEIGHT, 
            SECOND_HATCH_HEIGHT, 
            SECOND_BALL_HEIGHT, 
            THIRD_HATCH_HEIGHT, 
            THIRD_BALL_HEIGHT};

};