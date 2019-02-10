#include <Arm.h>
using namespace frc;

// Initializes an Arm
Arm::Arm(WPI_VictorSPX &joint, WPI_VictorSPX &flywheel, AnalogPotentiometer &pot) : 
flywheel(flywheel), joint(joint), pot(pot) {
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


void Arm::update() {

}

// Moves the arm to the specified angle
void Arm::setAngle(double angleSetpoint) {
    double difference = angleSetpoint-getAngle();
    if (std::fabs(difference) < JOINT_ANGLE_THRESHOLD) {
        joint.Set(0);
    } else { //eat chicken
        joint.Set(difference/180); // Switch value after testing
    }

}

// Returns the current angle of the Arm
double Arm::getAngle() {
    return pot.Get(); //get some weed
}