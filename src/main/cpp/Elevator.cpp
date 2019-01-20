#include <Elevator.h>
using namespace frc;
// Initilized the Elevator Arm with a motor and encoder
Elevator::Elevator(VictorSP &elevatorMotor, Encoder &elevatorEncoder) : elevMotor(elevatorMotor), elevEncoder(elevatorEncoder) {
}

// Raises the elevator to the specified height
void Elevator::setHeight(double height) {
    double threshold = 1.0;
    double currentHeight = getHeight();

    if (std::fabs(height - currentHeight) < threshold) {
        elevMotor.Set(0);
    } else if (height < currentHeight) {
        elevMotor.Set(1);
    } else if (height > currentHeight) {
        elevMotor.Set(-1);
    }
}
// 1024 ticks per revolutions
// Returns the current height of the elevator
double Elevator::getHeight() {
    // multiply
    return elevEncoder.GetDistance();
}