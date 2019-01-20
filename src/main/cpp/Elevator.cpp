#include <Elevator.h>

// Initilized the Elevator Arm with a motor and encoder
ElevatorArm::ElevatorArm(VictorSP &elevatorMotor, Encoder &elevatorEncoder) : elevMotor(elevatorMotor), elevEncoder(elevatorEncoder) {
    elevatorEncoder.SetDistancePerPulse(ENCODER_TICK_DISTANCE);
}

// Raises the elevator to the specified height
ElevatorArm::setHeight(double height) {
    int motorDirection = 1; // Raise Elevator
    double threshold = 1.0;
    double currentHeight = getHeight();

    if (height < currentHeight) { // Lower Elevator
        motorDirection = -1;
    }

    if (std::fabs(height - currentHeight) < threshold) { // Stop Elevator
        motorDirection = 0;
    }
    
    elevatorMotor.Set(motorDirection);
}
// 1024 ticks per revolutions
// Returns the current height of the elevator
ElevatorArm::getHeight()
{
    // multiply
    return elevEncoder.GetDistance();
}