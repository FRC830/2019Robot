#include <Lib830.h>
#include <ctre/Phoenix.h>
#include <frc/WPILib.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/BuiltInWidgets.h>
class Elevator {
    public:
        Elevator(WPI_TalonSRX &motor);
        double getHeight();
        void setSetpoint(int height);
        void setManualSpeed(double speed);
        int numSetpoints();
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
        
        // Heights of all targets, In inches
        nt::NetworkTableEntry nt_fhh, nt_shh, nt_thh, nt_fbh, nt_sbh, nt_tbh; // have fun deciphering this
        double first_hatch_height = 19 + SPEAR_OFFSET;
        double second_hatch_height = first_hatch_height + DIVIDER_DISTANCE;
        double third_hatch_height = second_hatch_height + DIVIDER_DISTANCE;

        double first_ball_height = 27.5 + FLYWHEEL_OFFSET;
        double second_ball_height = first_ball_height + DIVIDER_DISTANCE;
        double third_ball_height = second_ball_height + DIVIDER_DISTANCE;
        // defaults
        double p = 0.02 ,i = 0 ,d = 0,f = 0.0;

        bool motorFlipped = false;
        bool encoderFlipped = false;
        // network table stuff
        nt::NetworkTableEntry nt_p, nt_i, nt_d, nt_f, nt_motorFlipped, nt_encoderFlipped;

        std::vector<double> heights = {
            first_hatch_height, 
            first_ball_height, 
            second_hatch_height, 
            second_ball_height, 
            third_hatch_height, 
            third_ball_height};

};