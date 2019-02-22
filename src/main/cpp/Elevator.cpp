#include "Elevator.h"
using namespace frc;

// Initialized the Elevator Arm with a motor and encoder
Elevator::Elevator(WPI_TalonSRX &motor) : motor(motor) {
    motor.SetNeutralMode(NeutralMode::Brake);
    // practice has true for both
    Shuffleboard::GetTab("Robot Specific").AddPersistent("ELEV ENCODER FLIPPED", encoderFlipped)
    .WithWidget("Toggle Switch").GetEntry().GetBoolean(encoderFlipped);
    Shuffleboard::GetTab("Robot Specific").AddPersistent("ELEV MOTOR FLIPPED", motorFlipped)
    .WithWidget("Toggle Switch").GetEntry().GetBoolean(motorFlipped);
    
    Shuffleboard::GetTab("Robot Specific").AddPersistent("ELEV P", p).GetEntry().GetDouble(p);
    Shuffleboard::GetTab("Robot Specific").AddPersistent("ELEV I", i).GetEntry().GetDouble(i);
    Shuffleboard::GetTab("Robot Specific").AddPersistent("ELEV D", d).GetEntry().GetDouble(d);
    Shuffleboard::GetTab("Robot Specific").AddPersistent("ELEV F", f).GetEntry().GetDouble(f);
}
void Elevator::update() {
    motor.SetSensorPhase(SmartDashboard::GetBoolean("ELEV ENCODER FLIPPED", encoderFlipped));
    motor.SetInverted(SmartDashboard::GetBoolean("ELEV MOTOR FLIPPED", motorFlipped));
    motor.Config_kP(0, SmartDashboard::GetNumber("ELEV P", p));
    motor.Config_kI(0, SmartDashboard::GetNumber("ELEV I", i));
    motor.Config_kD(0, SmartDashboard::GetNumber("ELEV D", d));
    motor.Config_kF(0, SmartDashboard::GetNumber("ELEV F", f));
}
// Raises the elevator to the specified height
void Elevator::setSetpoint(int height) {
    // PID Controller
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