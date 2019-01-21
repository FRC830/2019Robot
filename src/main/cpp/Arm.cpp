#include <Arm.h>
using namespace frc;

// Initializes an Arm
Arm::Arm(VictorSP &joint, VictorSP &flywheel, AnalogPotentiometer &pot, Solenoid &piston) : 
flywheel(flywheel), joint(joint), pot(pot), piston(piston) {
    timer.Start();
}

// Turns the intake flywheels on or off
void Arm::setMode(Mode mode) {
    if (mode == OUTTAKE) {
        flywheel.Set(1);
    } else if (mode == INTAKE) {
        flywheel.Set(-1);
    }
}

// Extends Pistons
void Arm::releasePistons() {
    timer.Reset();
    piston.Set(true);
}

// Updates Pistons
void Arm::update() {
    if (timer.Get() > PISTON_EXTENSION_TIME) {
        piston.Set(false);
    }
}

// Moves the arm to the specified angle
void Arm::setAngle(double angle) {
    double currentAngle = getAngle();
    if (std::fabs(angle - currentAngle) < JOINT_ANGLE_THRESHOLD) {
        joint.Set(0);
    } else if (angle > currentAngle) {
        joint.Set(-1);
    } else if (angle < currentAngle) {
        joint.Set(1);
    }

}

// Returns the current angle of the Arm
double Arm::getAngle() {
    return pot.Get();
}
