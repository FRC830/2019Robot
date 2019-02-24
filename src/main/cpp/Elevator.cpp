#include "Elevator.h"
using namespace frc;

// Initialized the Elevator Arm with a motor and encoder
Elevator::Elevator(WPI_TalonSRX &motor) : motor(motor) {
    motor.SetNeutralMode(NeutralMode::Brake);
    // practice has true for both
    nt_encoderFlipped = Shuffleboard::GetTab("Robot Specific").AddPersistent("ELEV ENCODER FLIPPED", encoderFlipped)
    .WithWidget("Toggle Switch").GetEntry();
    nt_motorFlipped = Shuffleboard::GetTab("Robot Specific").AddPersistent("ELEV MOTOR FLIPPED", motorFlipped)
    .WithWidget("Toggle Switch").GetEntry();
    
    nt_p = Shuffleboard::GetTab("Robot Specific").AddPersistent("ELEV P", p).GetEntry();
    nt_i = Shuffleboard::GetTab("Robot Specific").AddPersistent("ELEV I", i).GetEntry();
    nt_d = Shuffleboard::GetTab("Robot Specific").AddPersistent("ELEV D", d).GetEntry();
    nt_f = Shuffleboard::GetTab("Robot Specific").AddPersistent("ELEV F", f).GetEntry();
}
void Elevator::update() {
    motor.SetSensorPhase(nt_encoderFlipped.GetBoolean(encoderFlipped));
    motor.SetInverted(nt_motorFlipped.GetBoolean(motorFlipped));
    motor.Config_kP(0, nt_p.GetDouble(p));
    motor.Config_kI(0, nt_i.GetDouble(i));
    motor.Config_kD(0, nt_d.GetDouble(d));
    motor.Config_kF(0, nt_f.GetDouble(f));
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