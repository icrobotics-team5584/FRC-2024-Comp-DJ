// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubAmp.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc2::cmd;

SubAmp::SubAmp() = default;

// This method will be called once per scheduler run
void SubAmp::Periodic(){
    frc::SmartDashboard::PutNumber("amp/Amp Shooter Motor: ", _AmpMotorSpin.Get());
}

frc2::CommandPtr SubAmp::AmpShooter(){
   return StartEnd(
    [this]{_AmpMotorSpin.Set(0.01);},
    [this]{_AmpMotorSpin.Set(0);}
   );

}

    
