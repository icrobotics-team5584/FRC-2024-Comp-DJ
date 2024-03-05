// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubLED.h"
#include <frc/AddressableLED.h>
#include "RobotContainer.h"
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>


using namespace frc2::cmd;

SubLED::SubLED() {

}
// This method will be called once per scheduler run
void SubLED::Periodic() {
}

void SubLED::SetLEDFunc(double PWMSIGNAL){
  _blinkin.Set(PWMSIGNAL);
}

frc2::CommandPtr SubLED::SetLEDCommand(double PWMSIGNAL) {
  return RunOnce([this, PWMSIGNAL]{_blinkin.Set(PWMSIGNAL);});
}



