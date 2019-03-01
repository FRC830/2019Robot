#include "Spear.h"
using namespace frc;


// Initializes an Spear
Spear::Spear(Solenoid &hatchGrabPiston, Solenoid &extensionPiston) : hatchGrabPiston(hatchGrabPiston), extensionPiston(extensionPiston) {

}

// Set the extension piston position (true is extended, false is retracted)
void Spear::setExtend (bool state) {
    extensionPiston.Set(state);
}

// Set the grab piston position (true is down, false is up)
void Spear::setHatchGrab(bool state) {
    hatchGrabPiston.Set(state);
}

// Toggle the place routine
void Spear::setPlaceRoutine(bool placing) {
    this->placing = placing;
    if (placing) {
        timer.Start();
    }
}

// Toggle the grab routine
void Spear::setGrabRoutine(bool grabbing) {
    this->grabbing = grabbing;
    if (grabbing) {
        timer.Start();
    }
}

// Update the placing & grabbing routine
void Spear::updateRoutine() {
    //stops both routines running at same time or when neither is running
    if (placing && grabbing) {
        timer.Stop();
        timer.Reset();
        return;
    }

    if (placing != previous_placing || grabbing != previous_grabbing) {
        timer.Reset();
        timer.Start();
    }


    double time = timer.Get();
    /* Placing Routine
    (1) Extend Arm
    (2) Flip Down
    WAIT
    (3) Retract Down
    (4) Flip Up*/
    if (placing) {
        setHatchGrab(false);
        setExtend(true);
    } else if (previous_placing || end_placing) {
        if (time < FLIP_DELAY) {
            end_placing = true;
            setHatchGrab(true);
            setExtend(true);
        } else if (time < FLIP_DELAY + AFTER_PLACE_FLIP_DELAY) {
            setHatchGrab(true);
            setExtend(false);
        } else{
            setHatchGrab(false);
            setExtend(false);
            end_placing = false;
        }
    }

    /* Grabbing Routine
    (1) Flip Down
    (2) Extend Arm
    (3) Flip Up
    (4) Retract*/

    if (grabbing) {
        if (time < FLIP_DELAY) {
            setHatchGrab(true);
            setExtend(false);
        } else {
            setHatchGrab(true);
            setExtend(true);
        }
    } else if (previous_grabbing || end_grabbing) {
        if (time < FLIP_DELAY) {
            end_grabbing = true;
            setHatchGrab(false);
            setExtend(true);
        } else {
            setHatchGrab(false);
            setExtend(false);
            end_grabbing = false;
        }
    }
    previous_grabbing = grabbing;
    previous_placing = placing;
}