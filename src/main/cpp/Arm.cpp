#include <Arm.h>
using namespace frc;

// Initializes an Arm
Arm::Arm(WPI_VictorSPX &joint, WPI_VictorSPX &flywheel, AnalogPotentiometer &pot, Solenoid &puncher) : 
flywheel(flywheel), joint(joint), pot(pot), puncher(puncher) {
    timer.Start();
}

// Turns the intake flywheels on or off
void Arm::setMode(Mode mode) {
    if (mode == OUTTAKE) {
        flywheel.Set(1);
    } else if (mode == INTAKE) {
        flywheel.Set(-1);
    } else if (mode == OFF) {
        flywheel.Set(0);
    }
}

// Extends Pistons
void Arm::releasePistons() {
    timer.Reset();
    puncher.Set(true);
}

// Updates Pistons
void Arm::update() {
    if (timer.Get() > PISTON_EXTENSION_TIME) {
        puncher.Set(false);
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
