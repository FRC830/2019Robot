#include <Lib830.h>
#include <frc/WPILib.h>
#include <ctre/Phoenix.h>

class Elevator {
    public:
    Elevator(WPI_VictorSPX &winch, frc::Encoder &elevatorEncoder);
    double getHeight();
    void setHeight(double height);
    private:
    WPI_VictorSPX &winch;
    frc::Encoder &elevatorEncoder;
    static constexpr double HEIGHT_THRESHOLD = 1.0;
};