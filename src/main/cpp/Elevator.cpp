#include "Elevator.h"
using namespace frc;

// Initialized the Elevator Arm with a motor and encoder
Elevator::Elevator(WPI_TalonSRX &motor) : motor(motor) {
    motor.SetNeutralMode(NeutralMode::Brake);
    // practice has true for both
    Shuffleboard::GetTab("robot-specific values").AddPersistent("[elev] Encoder Flipped", encoderFlipped)
    .WithWidget("Toggle Switch").GetEntry().GetBoolean(encoderFlipped);
    Shuffleboard::GetTab("robot-specific values").AddPersistent("[elev] Motor Flipped", motorFlipped)
    .WithWidget("Toggle Switch").GetEntry().GetBoolean(motorFlipped);
    
    Shuffleboard::GetTab("robot-specific values").AddPersistent("P [elev]", p).GetEntry().GetDouble(p);
    Shuffleboard::GetTab("robot-specific values").AddPersistent("I [elev]", i).GetEntry().GetDouble(i);
    Shuffleboard::GetTab("robot-specific values").AddPersistent("D [elev]", d).GetEntry().GetDouble(d);
    Shuffleboard::GetTab("robot-specific values").AddPersistent("F [elev]", f).GetEntry().GetDouble(f);
}

// Raises the elevator to the specified height
void Elevator::setSetpoint(int height) {
    // PID Controller
    motor.SetSensorPhase(SmartDashboard::GetBoolean("[elev] Encoder Flipped", encoderFlipped));
    motor.SetInverted(SmartDashboard::GetBoolean("[elev] Motor Flipped", motorFlipped));
    motor.Config_kP(0, SmartDashboard::GetNumber("P [elev]", p));
    motor.Config_kI(0, SmartDashboard::GetNumber("I [elev]", i));
    motor.Config_kD(0, SmartDashboard::GetNumber("D [elev]", d));
    motor.Config_kF(0, SmartDashboard::GetNumber("F [elev]", f));
    motor.Set(ControlMode::Position, heights[height] / ENCODER_TICK_DISTANCE);
}

void Elevator::setManualSpeed(double speed) {
    motor.SetSensorPhase(SmartDashboard::GetBoolean("[elev] Encoder Flipped", encoderFlipped));
    motor.SetInverted(SmartDashboard::GetBoolean("[elev] Motor Flipped", motorFlipped));
    motor.Set(ControlMode::PercentOutput, speed);
}

double Elevator::getHeight() {
    return motor.GetSelectedSensorPosition() * ENCODER_TICK_DISTANCE;
}

int Elevator::numSetpoints() {
    return heights.size();
}