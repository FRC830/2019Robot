#include "Elevator.h"
using namespace frc;

// Initialized the Elevator Arm with a motor and encoder
Elevator::Elevator(WPI_VictorSPX &winch, Encoder &elevatorEncoder) : winch(winch), elevatorEncoder(elevatorEncoder) {

}

// Raises the elevator to the specified height
void Elevator::setHeight(double height) {
    double currentHeight = getHeight();
    if (std::fabs(height - currentHeight) < HEIGHT_THRESHOLD) {
        winch.Set(0);
    } else if (height < currentHeight) {
        winch.Set(1);
    } else if (height > currentHeight) {
        winch.Set(-1);
    }
}

// Returns the current height of the elevator
double Elevator::getHeight() {
    return elevatorEncoder.GetDistance();
}