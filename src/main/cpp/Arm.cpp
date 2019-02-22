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
    Shuffleboard::GetTab("robot-specific values").AddPersistent("inside_frame_perimeter", inside_frame_perimeter).GetEntry().GetDouble(inside_frame_perimeter);
    Shuffleboard::GetTab("robot-specific values").AddPersistent("ball_height", ball_height).GetEntry().GetDouble(ball_height);
    Shuffleboard::GetTab("robot-specific values").AddPersistent("spear_height", spear_height).GetEntry().GetDouble(spear_height);
    Shuffleboard::GetTab("robot-specific values").AddPersistent("intake_height", intake_height).GetEntry().GetDouble(intake_height);

    Shuffleboard::GetTab("robot-specific values").AddPersistent("P [arm]", p).GetEntry().GetDouble(p);
    Shuffleboard::GetTab("robot-specific values").AddPersistent("I [arm]", i).GetEntry().GetDouble(i);
    Shuffleboard::GetTab("robot-specific values").AddPersistent("D [arm]", d).GetEntry().GetDouble(d);
    Shuffleboard::GetTab("robot-specific values").AddPersistent("F [arm]", f).GetEntry().GetDouble(f);
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
    armPID.SetSetpoint(armAngles[index]);
}

// Returns the current angle of the Arm
double Arm::getAngle() {
    armAngles[0] = SmartDashboard::GetNumber("inside_frame_perimeter", inside_frame_perimeter);
    armAngles[1] = SmartDashboard::GetNumber("ball_height", ball_height);
    armAngles[2] = SmartDashboard::GetNumber("spear_height", spear_height);
    armAngles[3] = SmartDashboard::GetNumber("intake_height", intake_height);

    armPID.SetP(SmartDashboard::GetNumber("P [arm]",p));
    armPID.SetI(SmartDashboard::GetNumber("I [arm]",i));
    armPID.SetD(SmartDashboard::GetNumber("D [arm]",d));
    armPID.SetF(SmartDashboard::GetNumber("F [arm]",f));
    
    return pot.Get();
}
// Returns the number of setpoints
int Arm::numSetpoints() {
    return armAngles.size();
}