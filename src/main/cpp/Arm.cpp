#include <Arm.h>

// Initializes an Arm
Arm(VictorSP &armMotor, AnalogPotentiometer &armPotentiometer, VictorSP &flywheelMotor): 
flywheel(flywheelMotor), arm(armMotor), armPot(armPotentiometer) {

}
// Turns the intake flywheels on or off
Arm::setIntake(Mode mode) {

}

// Pushes the intake pistons out and retracts
Arm::releasePistons() {

}

// Moves the arm to the specified angle
Arm::setAngle(float angle) {
    double currentAngle = getAngle();
    
}

// Returns the current angle of the Arm
Arm::getAngle() {
    return armPot.Get();
}
