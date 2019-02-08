#include <Lib830.h>
#include <frc/WPILib.h>
#include <ctre/Phoenix.h>

class Elevator {
    public:
        Elevator(WPI_VictorSPX &motor);
        double getHeight();
        void setHeight(int height);
        void setManualSpeed(double speed);
        int numSetpoints();
    private:
        WPI_VictorSPX &motor;

        static constexpr double HEIGHT_THRESHOLD = 1.0;
        static constexpr double CARGO_HEIGHT = 10.0; 
        static constexpr double FIRST_BALL_HEIGHT = 20.0;
        static constexpr double SECOND_HATCH_HEIGHT = 30.0;
        static constexpr double SECOND_BALL_HEIGHT = 40.0;
        static constexpr double THIRD_HATCH_HEIGHT = 50.0;
        static constexpr double THIRD_BALL_HEIGHT = 60.0;

        std::vector<double> heights = {
            CARGO_HEIGHT, 
            FIRST_BALL_HEIGHT, 
            SECOND_HATCH_HEIGHT, 
            SECOND_BALL_HEIGHT, 
            THIRD_HATCH_HEIGHT, 
            THIRD_BALL_HEIGHT};

};