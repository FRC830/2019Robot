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
    // Shuffleboard::GetTab("values").AddPersistent("Arm PID", &armPID).GetObject(armPID);
    Shuffleboard::GetTab("Robot Specific").AddPersistent("INSIDE FRAME", inside_frame_perimeter).GetEntry().GetDouble(inside_frame_perimeter);
    Shuffleboard::GetTab("Robot Specific").AddPersistent("BALL OUTTAKE", ball_height).GetEntry().GetDouble(ball_height);
    Shuffleboard::GetTab("Robot Specific").AddPersistent("SPEAR ANGLE", spear_height).GetEntry().GetDouble(spear_height);
    Shuffleboard::GetTab("Robot Specific").AddPersistent("BALL INTAKE", intake_height).GetEntry().GetDouble(intake_height);

    Shuffleboard::GetTab("Robot Specific").AddPersistent("ARM P", p).GetEntry().GetDouble(p);
    Shuffleboard::GetTab("Robot Specific").AddPersistent("ARM I", i).GetEntry().GetDouble(i);
    Shuffleboard::GetTab("Robot Specific").AddPersistent("ARM D", d).GetEntry().GetDouble(d);
    Shuffleboard::GetTab("Robot Specific").AddPersistent("ARM F", f).GetEntry().GetDouble(f);
}
void Arm::update() {
    armAngles[0] = SmartDashboard::GetNumber("INSIDE FRAME", inside_frame_perimeter);
    armAngles[1] = SmartDashboard::GetNumber("BALL OUTTAKE", ball_height);
    armAngles[2] = SmartDashboard::GetNumber("SPEAR ANGLE", spear_height);
    armAngles[3] = SmartDashboard::GetNumber("BALL INTAKE", intake_height);

    armPID.SetP(SmartDashboard::GetNumber("ARM P", p));
    armPID.SetI(SmartDashboard::GetNumber("ARM I", i));
    armPID.SetD(SmartDashboard::GetNumber("ARM D", d));
    armPID.SetF(SmartDashboard::GetNumber("ARM F", f));
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