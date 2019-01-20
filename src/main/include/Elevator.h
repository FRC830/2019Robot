#include <Lib830.h>
#include <frc/WPILib.h>

// This class controls the movement of the 3-stage cascade elevator
class Elevator {
    public:
    Elevator(VictorSP *ElevatorMotor, Encoder *ElevatorEncoder);
    float getHeight();
    void setHeight(height float);
    private:

    static const int ENCODER_TICKS = 1024;
    static const float PI = 3.1415927;
    static const int WINCH_DIAMETER = 6; // PLACEHOOLDER;
    static const int ENCODER_TICK_DISTANCE = 6 * PI / ENCODER_TICKS;

    VictorSP elevMotor{ELEVATOR_MOTOR_PORT};
    Encoder elevEncoder{ELEVATOR_ENCODER_DIO_ONE, ELEVATOR_ENCODER_DIO_TWO};
}