// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/SubAmp.h"

using namespace frc2::cmd;

SubAmp::SubAmp(){
    _ampMotorSpin.RestoreFactoryDefaults();
}

// This method will be called once per scheduler run
void SubAmp::Periodic(){
    frc::SmartDashboard::PutNumber("amp/Amp Shooter Motor: ", _ampMotorSpin.Get());
}

frc2::CommandPtr SubAmp::AmpShooter(){
   return StartEnd(
    [this]{_ampMotorSpin.Set(0.5);},
    [this]{_ampMotorSpin.Set(0);}
   );
}

frc2::CommandPtr SubAmp::ReverseAmpShooter(){
    return StartEnd(
    [this]{_ampMotorSpin.Set(-0.5);},
    [this]{_ampMotorSpin.Set(0);}
   );
}

/*
constexpr int ClawMotorJoint = 100;
constexpr int ElevatorMotor = 101;
constexpr int AmpMotorSpin = 1

frc2::CommandPtr CubeConeSwitch(){
        return RunOnce([]{
            if(RobotContainer::isConeMode){RobotContainer::isConeMode = false;}
            else{RobotContainer::isConeMode = true;}          
        });
    }


frc2::CommandPtr SubAmp::ClawTiltDown(){
    return RunOnce(
        []{  }
    );
}
frc2::CommandPtr SubAmp::ClawTiltUp(){}

*/