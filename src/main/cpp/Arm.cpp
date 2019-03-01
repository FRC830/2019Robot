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

    for (int i = 0; i < ntAngles.size(); i++) {
        ntAngles[i] = robotConfigTab.AddPersistent(armHeightWords[i], defaultAngles[i]).GetEntry();
    }

    nt_p = robotConfigTab.AddPersistent("ARM P", p).GetEntry();
    nt_i = robotConfigTab.AddPersistent("ARM I", i).GetEntry();
    nt_d = robotConfigTab.AddPersistent("ARM D", d).GetEntry();
    nt_f = robotConfigTab.AddPersistent("ARM F", f).GetEntry();
}

// Refresh configuration
void Arm::update() {
    for (int i = 0; i < angles.size(); i++) {
        angles[i] = ntAngles[i].GetDouble(defaultAngles[i]);
    }

    armPID.SetP(nt_p.GetDouble(p));
    armPID.SetI(nt_i.GetDouble(i));
    armPID.SetD(nt_d.GetDouble(d));
    armPID.SetF(nt_f.GetDouble(f));
}

// Turns the intake flywheels on or off
void Arm::setMode(FlywheelMode mode) {
    flywheel.Set(mode);
}

// Set the arm motor speed manually
void Arm::setManualSpeed(double speed) {
    if (armPID.IsEnabled()) {
        armPID.Disable();
    }
    joint.Set(speed);
}

// Moves the arm to the specified angle
void Arm::changeSetpoint(int change) {
    if ((0 <= currentSetpoint + change) && (currentSetpoint + change <= angles.size() - 1)) {
        currentSetpoint += change;
    }
}
void Arm::setSetpoint() {
    if (!armPID.IsEnabled()) {
        armPID.Enable();
    }
    armPID.SetSetpoint(angles[currentSetpoint]);
}
// Returns the current angle of the Arm
double Arm::getAngle() {
    return pot.Get();
}
// Returns the number of setpoints
std::string Arm::getSetpoint() {
    return armHeightWords[currentSetpoint];
}