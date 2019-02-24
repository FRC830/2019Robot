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

    // nt_fhh=robotConfigTab.AddPersistent("BOTTOM HATCH", first_hatch_height).GetEntry();
    // nt_fbh=robotConfigTab.AddPersistent("BOTTOM BALL", first_ball_height).GetEntry();
    // nt_shh=robotConfigTab.AddPersistent("MIDDLE HATCH", second_hatch_height).GetEntry();
    // nt_sbh=robotConfigTab.AddPersistent("MIDDLE BALL", second_ball_height).GetEntry();
    // nt_thh=robotConfigTab.AddPersistent("TOP BALL", third_hatch_height).GetEntry();
    // nt_tbh=robotConfigTab.AddPersistent("TOP HATCH", third_ball_height).GetEntry();
}
void Elevator::update() {

    // heights[0] = nt_fhh.GetDouble(first_hatch_height);
    // heights[1] = nt_fbh.GetDouble(first_ball_height);
    // heights[2] = nt_shh.GetDouble(second_hatch_height);
    // heights[3] = nt_sbh.GetDouble(second_ball_height);
    // heights[4] = nt_thh.GetDouble(third_hatch_height);
    // heights[5] = nt_tbh.GetDouble(third_ball_height);

    motor.ConfigPeakOutputReverse(nt_max_down.GetDouble(max_down_speed)); // may need to be ConfigPeakOutputForward
    motor.SetSensorPhase(nt_encoderFlipped.GetBoolean(encoderFlipped));
    motor.SetInverted(nt_motorFlipped.GetBoolean(motorFlipped));
    motor.Config_kP(0, nt_p.GetDouble(p));
    motor.Config_kI(0, nt_i.GetDouble(i));
    motor.Config_kD(0, nt_d.GetDouble(d));
    motor.Config_kF(0, nt_f.GetDouble(f));
}
// Raises the elevator to the specified height
void Elevator::setSetpoint(int height) {
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