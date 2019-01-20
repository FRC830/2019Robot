#include <Arm.h>
using namespace frc;
// Initializes an Arm
Arm::Arm(VictorSP &armMotor, VictorSP &flywheelMotor, AnalogPotentiometer &armPotentiometer) : flywheel(flywheelMotor), arm(armMotor), armPot(armPotentiometer)
{
}
// Turns the intake flywheels on or off
void Arm::setIntake(bool mode) {

}

// Pushes the intake pistons out and retracts
void Arm::releasePistons() {

}

// Moves the arm to the specified angle
void Arm::setAngle(double angle) {
    double currentAngle = getAngle();
    double threshold = 5.0;
    if (std::fabs(angle - currentAngle) < threshold) {
        arm.Set(0);
    } else if (angle > currentAngle) {
        arm.Set(-1);
    } else if (angle < currentAngle) {
        arm.Set(1);
    }

}

// Returns the current angle of the Arm
double Arm::getAngle() {
    return armPot.Get();
}
