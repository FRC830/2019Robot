#include "Spear.h"
using namespace frc;

// Initializes an Spear
Spear::Spear(Solenoid &hatchGrabPiston, Solenoid &extensionPiston) : 
hatchGrabPiston(hatchGrabPiston), extensionPiston(extensionPiston) {
    timer.Start();
}

void Spear::setExtend (bool state) {
    extensionPiston.Set(state);
}

void Spear::setHatchGrab (bool state) {
    hatchGrabPiston.Set(state);
}

void Spear::placeRoutine(bool placing){
    placing = placing;
}

void Spear::grabRoutine(bool grabbing){
    grabbing = grabbing;
}

void Spear::updateRoutine(){
    if (running || grabbing){
        
    }
}
// Hatch(Automated)

//Intake (A Button?) Active VR. Inactive

//JoystickButton (JoystickButton &&A)=default, virtual void 
//frc::Button::ToggleWhenPressed( hatchGrabPiston.Set(false) || extensionPiston.Set(true)
// *hatchGrabPiston.Set(true) || extensionPiston.Set(false))	

//Press 1: Grab | Spearhead moves down (Pneumatic 1 recoils), Wait for completion, Shaft Extends (Pneumatic 2 extends)
//Press 2: Secure | Spearhead moves up (Pneumatic 1 extends), Wait for completion, Shaft Retracts (Pneumatic 2 recoils)

//Outtake (B Button//?) 

//JoystickButton (JoystickButton &&B)=default, virtual void 
//frc::Button::ToggleWhenPressed(hatchGrabPiston.Set(false) || extensionPiston.Set(false) * command	)	

//Press 1: Extend | Shaft Extends (Pneumatics 2 Extends) 
//Press 2: Release | Spearhead moves down (Pneumatic 1 recoils), Wait for completion, Shaft Retracts (Pneumatic 2 recoils), 
//Spearhead moves up (Pneumatic 1 extends