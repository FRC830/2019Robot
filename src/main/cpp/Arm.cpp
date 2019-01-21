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

// Pushes the intake pistons out and retracts
void Arm::releasePistons() {
    timer.Reset();
    piston.Set(true);
}
void Arm::update() {
    if (timer.Get() > 0.5) {
        piston.Set(false);
    }
}
// Moves the arm to the specified angle
void Arm::setAngle(double angle) {
    double currentAngle = getAngle();
    double threshold = 5.0;
    if (std::fabs(angle - currentAngle) < threshold) {
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
