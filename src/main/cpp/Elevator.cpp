#include "Elevator.h"
using namespace frc;

// Initialized the Elevator Arm with a motor and encoder
Elevator::Elevator(WPI_TalonSRX &motor) : motor(motor) {
    motor.SetNeutralMode(NeutralMode::Brake);
    ShuffleboardTab& robotConfigTab = Shuffleboard::GetTab("Robot Specific");

    nt_encoderFlipped = robotConfigTab.AddPersistent("ELEV ENCODER FLIPPED", encoderFlipped).WithWidget("Toggle Switch").GetEntry();
    nt_motorFlipped = robotConfigTab.AddPersistent("ELEV MOTOR FLIPPED", motorFlipped).WithWidget("Toggle Switch").GetEntry();
    
    nt_p = robotConfigTab.AddPersistent("ELEV P", p).GetEntry();
    nt_i = robotConfigTab.AddPersistent("ELEV I", i).GetEntry();
    nt_d = robotConfigTab.AddPersistent("ELEV D", d).GetEntry();
    nt_f = robotConfigTab.AddPersistent("ELEV F", f).GetEntry();

    nt_max_down=robotConfigTab.AddPersistent("ELEV MAX DOWN", max_down_speed).GetEntry();

    // for (int i = 0; i < ntHeights.size(); i++) {
    //     ntHeights[i] = robotConfigTab.AddPersistent(elevatorHeightWords[i], defaultHeights[i]).GetEntry();
    // }
}
void Elevator::update() {
    // for (int i = 0; i < heights.size(); i++) {
    //     heights[i] = ntHeights[i].GetDouble(defaultHeights[i]);
    // }

    motor.ConfigPeakOutputReverse(nt_max_down.GetDouble(max_down_speed)); // may need to be ConfigPeakOutputForward
    motor.SetSensorPhase(nt_encoderFlipped.GetBoolean(encoderFlipped));
    motor.SetInverted(nt_motorFlipped.GetBoolean(motorFlipped));
    motor.Config_kP(0, nt_p.GetDouble(p));
    motor.Config_kI(0, nt_i.GetDouble(i));
    motor.Config_kD(0, nt_d.GetDouble(d));
    motor.Config_kF(0, nt_f.GetDouble(f));
}
// Raises the elevator to the specified height
void Elevator::changeSetpoint(int change) {
    if ((0 <= currentSetpoint + change) && (currentSetpoint + change <= heights.size() - 1)) {
        currentSetpoint += change;
    }
    motor.Set(ControlMode::Position, heights[currentSetpoint] / ENCODER_TICK_DISTANCE);
}
void Elevator::setManualSpeed(double speed) {
    motor.Set(ControlMode::PercentOutput, speed);
}

double Elevator::getHeight() {
    return motor.GetSelectedSensorPosition() * ENCODER_TICK_DISTANCE;
}

std::string Elevator::getSetpoint() {
    return elevatorHeightWords[currentSetpoint];
}