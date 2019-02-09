#include "Elevator.h"
using namespace frc;

// Initialized the Elevator Arm with a motor and encoder
Elevator::Elevator(WPI_VictorSPX &motor) : motor(motor) {
    motor.SetNeutralMode(NeutralMode::Brake);
    motor.SetSensorPhase(false);
    // PID Controller
    motor.Config_kP(0, 2.0);
    motor.Config_kI(0, 0.0);
    motor.Config_kD(0, 0.0);
    motor.Config_kF(0, 0.0);
}

// Raises the elevator to the specified height
void Elevator::setSetpoint(int height) {
    motor.Set(ControlMode::Position, heights[height] / ENCODER_TICK_DISTANCE);
}

void Elevator::setManualSpeed(double speed) {
    motor.Set(ControlMode::PercentOutput, speed);
}

int Elevator::numSetpoints() {
    return heights.size();
}