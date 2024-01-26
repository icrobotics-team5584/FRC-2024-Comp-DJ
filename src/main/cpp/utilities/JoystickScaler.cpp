#include "Utilities/JoystickScaler.h"
#include "math.h"

JoystickScaler::JoystickScaler(int deviceID, double scaleLeft, double scaleRight) :
    XboxController(deviceID)
{
    _scaleLeft = scaleLeft;
    _scaleRight = scaleRight;
}

double JoystickScaler::GetScaledX(Hand hand) {

    double controllerValue = 0;
    if (hand == left) { 
        _scale = _scaleLeft;
        controllerValue = XboxController::GetLeftX(); 
    }
    if (hand == right) { 
        _scale = _scaleRight; 
        controllerValue = XboxController::GetRightX();
    }
    


    if (controllerValue < 0) {
        return pow(abs(controllerValue), _scale);
    }
    else {
        return -1 * pow(abs(controllerValue), _scale);
    }
}

double JoystickScaler::GetScaledY(Hand hand) {

    double controllerValue = 0;
    if (hand == left) { 
        _scale = _scaleLeft;
        controllerValue = XboxController::GetLeftY(); 
    }
    if (hand == right) { 
        _scale = _scaleRight; 
        controllerValue = XboxController::GetRightY();
    }


    if (controllerValue < 0) {
        return -1 * pow(abs(controllerValue), _scale);
    }
    else {
        return pow(abs(controllerValue), _scale);
    }
}

void JoystickScaler::SetLeftScale(double scale) {
    _scaleLeft = scale;
}

void JoystickScaler::SetRightScale(double scale) {
    _scaleRight = scale;
}