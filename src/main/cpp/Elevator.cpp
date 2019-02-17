#include "Elevator.h"
using namespace frc;

// Initialized the Elevator Arm with a motor and encoder
Elevator::Elevator(WPI_TalonSRX &motor) : motor(motor) {
    motor.SetNeutralMode(NeutralMode::Brake);
    motor.SetSensorPhase(true); // flip encoder
    motor.SetInverted(true); // flip motor
    SmartDashboard::PutNumber("P",.02);
    SmartDashboard::PutNumber("I", 0.0);
    SmartDashboard::PutNumber("D", 0.0);
    SmartDashboard::PutNumber("F", 0.0);
    // PID Controller
}

// Raises the elevator to the specified height
void Elevator::setSetpoint(int height) {
    motor.Config_kP(0, SmartDashboard::GetNumber("P",.02));
    motor.Config_kI(0, SmartDashboard::GetNumber("I",0.0));
    motor.Config_kD(0, SmartDashboard::GetNumber("D",0.0));
    motor.Config_kF(0, SmartDashboard::GetNumber("F",0.0));
    motor.Set(ControlMode::Position, heights[height] / ENCODER_TICK_DISTANCE);
}

void Elevator::setManualSpeed(double speed) {
    motor.Set(ControlMode::PercentOutput, speed);
}

double Elevator::getHeight() {
    return motor.GetSelectedSensorPosition() * ENCODER_TICK_DISTANCE;
}

int Elevator::numSetpoints() {
    return heights.size();
}