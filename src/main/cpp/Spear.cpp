#include "Spear.h"
using namespace frc;

// Initializes an Spear
Spear::Spear(Solenoid &hatchGrabPiston, Solenoid &extensionPiston) : hatchGrabPiston(hatchGrabPiston), extensionPiston(extensionPiston) {

}

//true is extended, false is retracted
void Spear::setExtend (bool state) {
    extensionPiston.Set(state);
}

//true is down, false is up
void Spear::setHatchGrab (bool state) {
    hatchGrabPiston.Set(state);
}

void Spear::setPlaceRoutine(bool placing) {
    this->placing = placing;
    if (placing) {
        timer.Start();
    }
}

void Spear::setGrabRoutine(bool grabbing) {
    this->grabbing = grabbing;
    if (grabbing) {
        timer.Start();
    }
}

void Spear::updateRoutine() {
    //stops both routines running at same time or when neither is running
    if (placing == grabbing) {
        timer.Stop();
        timer.Reset();
        return;
    }

    double time = timer.Get();

    // Placing Routine
    // (1) Extend Arm
    // (2) Flip Down
    // (3) Retract Down
    // (4) Flip Up
    if (placing) {
        if (time < PLACING_EXTEND_TIME) {
            setHatchGrab(false);
            setExtend(true);
        } else if (time < PLACING_EXTEND_TIME + PLACING_FLIP_DOWN_TIME) {
            setHatchGrab(true);
            setExtend(true);
        } else if (time < PLACING_EXTEND_TIME + PLACING_FLIP_DOWN_TIME + PLACING_RETRACT_TIME) {
            setHatchGrab(true);
            setExtend(false);
        } else {
            setHatchGrab(false);
            setExtend(false);
        }                 
    }

    // Grabbing Routine
    // (1) Flip Down
    // (2) Extend Arm
    // (3) Flip Up
    // (4) Retract
    if (grabbing) {
        if (time < GRABBING_FLIP_DOWN_TIME) {
            setHatchGrab(true);
            setExtend(false);
        } else if (time < GRABBING_FLIP_DOWN_TIME + GRABBING_EXTEND_TIME) {
            setHatchGrab(true);
            setExtend(true);
        } else if (time < GRABBING_FLIP_DOWN_TIME + GRABBING_EXTEND_TIME + GRABBING_FLIP_UP_TIME) {
            setHatchGrab(false);
            setExtend(true);
        } else {
            setHatchGrab(false);
            setExtend(false);            
        }                 
    }
}