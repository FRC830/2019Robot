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
    SmartDashboard::PutData(&armPID);
    if (!armPID.IsEnabled()) {
        armPID.Enable();
    }
    armPID.SetSetpoint(armHeights[index]);
}

// Returns the current angle of the Arm
double Arm::getAngle() {
    return pot.Get();
}
// Returns the number of setpoints
int Arm::numSetpoints() {
    return armHeights.size();
}