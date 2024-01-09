// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/angle.h>

#include "subsystems/SubAmp.h"

using namespace frc2::cmd;

SubAmp::SubAmp(){
    _ampMotorSpin.RestoreFactoryDefaults();
}

// This method will be called once per scheduler run
void SubAmp::Periodic(){
    frc::SmartDashboard::PutNumber("amp/Amp Shooter Motor: ", _ampMotorSpin.Get());
    frc::SmartDashboard::PutNumber("amp/Dizzy Claw tilt motor speed: ", _clawMotorJoint.Get());
}


// Shooter Amp

frc2::CommandPtr SubAmp::AmpShooter(){
   return StartEnd(
    [this]{_ampMotorSpin.Set(0.7);},
    [this]{_ampMotorSpin.Set(0);}
   );
}

frc2::CommandPtr SubAmp::ReverseAmpShooter(){
    return StartEnd(
        [this]{_ampMotorSpin.Set(-0.7);},
        [this]{_ampMotorSpin.Set(0);}
    );
}

// dizzy amp

frc2::CommandPtr SubAmp::ClawOpen(){
    
}

frc2::CommandPtr SubAmp::ClawClose(){}

frc2::CommandPtr SubAmp::ClawTiltDown(){
    return RunOnce([this]{_clawMotorJoint.Set(-0.7);});
}

frc2::CommandPtr SubAmp::ClawTiltUp(){
    return RunOnce([this]{_clawMotorJoint.Set(0.7);});
}
