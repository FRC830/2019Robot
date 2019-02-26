#include <Arm.h>
using namespace frc;

// Initializes an Arm
Arm::Arm(WPI_VictorSPX &joint, WPI_VictorSPX &flywheel, AnalogPotentiometer &pot) : flywheel(flywheel), joint(joint), pot(pot) {
    joint.SetNeutralMode(NeutralMode::Brake);
    flywheel.SetInverted(true);
    armPID.SetInputRange(0, 300);
    armPID.SetOutputRange(-MAX_UP_SPEED, MAX_DOWN_SPEED);
    armPID.SetAbsoluteTolerance(JOINT_ANGLE_THRESHOLD);
    armPID.Enable();

    ShuffleboardTab& robotConfigTab = Shuffleboard::GetTab("Robot Specific");
    nt_inside_frame_perimeter = robotConfigTab.AddPersistent("INSIDE FRAME", inside_frame_perimeter).GetEntry();
    nt_ball_outtake = robotConfigTab.AddPersistent("BALL OUTTAKE", ball_outtake).GetEntry();
    nt_spear_angle = robotConfigTab.AddPersistent("SPEAR ANGLE", spear_angle).GetEntry();
    nt_spear_intake = robotConfigTab.AddPersistent("SPEAR INTAKE", spear_intake).GetEntry();
    nt_ball_intake = robotConfigTab.AddPersistent("BALL INTAKE", ball_intake).GetEntry();

    nt_p = robotConfigTab.AddPersistent("ARM P", p).GetEntry();
    nt_i = robotConfigTab.AddPersistent("ARM I", i).GetEntry();
    nt_d = robotConfigTab.AddPersistent("ARM D", d).GetEntry();
    nt_f = robotConfigTab.AddPersistent("ARM F", f).GetEntry();
}
void Arm::update() {
    armAngles[0] = nt_inside_frame_perimeter.GetDouble(inside_frame_perimeter);
    armAngles[1] = nt_ball_outtake.GetDouble(ball_outtake);
    armAngles[2] = nt_spear_angle.GetDouble(spear_angle);
    armAngles[3] = nt_spear_intake.GetDouble(spear_intake);
    armAngles[4] = nt_ball_intake.GetDouble(ball_intake);

    armPID.SetP(nt_p.GetDouble(p));
    armPID.SetI(nt_i.GetDouble(i));
    armPID.SetD(nt_d.GetDouble(d));
    armPID.SetF(nt_f.GetDouble(f));
}
// Turns the intake flywheels on or off
void Arm::setMode(FlywheelMode mode) {
    if (mode == OUTTAKE) {
        flywheel.Set(1);
    } else if (mode == INTAKE) {
        flywheel.Set(-1);
    } else if (mode == OFF) {
        flywheel.Set(0);
    }
}

void Arm::setManualSpeed(double speed) {
    if (armPID.IsEnabled()) {
        armPID.Disable();
    }
    joint.Set(speed);
}

// Moves the arm to the specified angle
void Arm::setAngle(int index) {
    if (!armPID.IsEnabled()) {
        armPID.Enable();
    }
    armPID.SetSetpoint(armAngles[index]);
}

// Returns the current angle of the Arm
double Arm::getAngle() {
    return pot.Get();
}
// Returns the number of setpoints
int Arm::numSetpoints() {
    return armAngles.size();
}