#include <Lib830.h>
#include <frc/WPILib.h>

class Elevator {
    public:
    Elevator(frc::VictorSP &winch, frc::Encoder &elevatorEncoder);
    double getHeight();
    void setHeight(double height);
    private:
    frc::VictorSP &winch;
    frc::Encoder &elevatorEncoder;
    static constexpr double HEIGHT_THRESHOLD = 1.0;
};