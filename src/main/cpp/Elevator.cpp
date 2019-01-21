#include <Elevator.h>
using namespace frc;
// Initilized the Elevator Arm with a motor and encoder
Elevator::Elevator(VictorSP &winch, Encoder &elevatorEncoder) : winch(winch), elevatorEncoder(elevatorEncoder) {

}

// Raises the elevator to the specified height
void Elevator::setHeight(double height) {
    double threshold = 1.0;
    double currentHeight = getHeight();

    if (std::fabs(height - currentHeight) < threshold) {
        winch.Set(0);
    } else if (height < currentHeight) {
        winch.Set(1);
    } else if (height > currentHeight) {
        winch.Set(-1);
    }
}
// 1024 ticks per revolutions
// Returns the current height of the elevator
double Elevator::getHeight() {
    // multiply
    return elevatorEncoder.GetDistance();
}