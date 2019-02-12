#include <Arm.h>
using namespace frc;

// Initializes an Arm
Arm::Arm(WPI_VictorSPX &joint, WPI_VictorSPX &flywheel, AnalogPotentiometer &pot) :
flywheel(flywheel), joint(joint), pot(pot) {
    timer.Start();
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

void Arm::setManualSpeed(double speed){
    joint.Set(speed);
}

// Moves the arm to the specified angle
void Arm::setAngle(int index) {
    double difference = armHeights[index]-getAngle();
    if (std::fabs(difference) < JOINT_ANGLE_THRESHOLD) {
        joint.Set(0);
    } else {
        joint.Set(difference/180); // Switch value after testing
    }

}

// Returns the current angle of the Arm
double Arm::getAngle() {
    return pot.Get();
}
// Returns the number of setpoints
int Arm::numSetpoints() {
    return armHeights.size();
}