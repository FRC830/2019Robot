#include <Lib830.h>
#include <frc/WPILib.h>

// This class controls the movement of the 3-stage cascade elevator
class Elevator {
    public:
    Elevator(frc::VictorSP &winch, frc::Encoder &elevatorEncoder);
    double getHeight();
    void setHeight(double height);
    private:
    frc::VictorSP &winch;
    frc::Encoder &elevatorEncoder;
};