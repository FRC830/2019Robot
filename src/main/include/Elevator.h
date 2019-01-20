#include <Lib830.h>
#include <frc/WPILib.h>

// This class controls the movement of the 3-stage cascade elevator
class Elevator {
    public:
    Elevator(frc::VictorSP &ElevatorMotor, frc::Encoder &ElevatorEncoder);
    double getHeight();
    void setHeight(double height);
    private:
    enum Heights{HATCH_CARGO_SHIP, BALL_CARGO_SHIP, ROCKET_LEVEL_TWO, ROCKET_LEVEL_THREE};
    frc::VictorSP &elevMotor;
    frc::Encoder &elevEncoder;
};