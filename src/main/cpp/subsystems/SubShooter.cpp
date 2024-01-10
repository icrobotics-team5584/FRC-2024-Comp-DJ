// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubShooter.h"
#include <frc/smartdashboard/SmartDashboard.h>



SubShooter::SubShooter(){
    _shooterMotorFollowerSpin.RestoreFactoryDefaults();
    _shooterMotorMainSpin.RestoreFactoryDefaults();

    _shooterMotorFollowerSpin.SetInverted(true);
}

using namespace frc2::cmd;




// This method will be called once per scheduler run
void SubShooter::Periodic() {
    frc::SmartDashboard::PutNumber("Shooter/Shooter Main Shooter Motor: ", _shooterMotorMainSpin.Get());
    frc::SmartDashboard::PutNumber("Shooter/Shooter Follower Shooter Motor: ", _shooterMotorMainSpin.Get());
    frc::SmartDashboard::PutNumber("Shooter/Main Motor Velocity", _shooterMainEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Shooter/Follower Motor Velocity", _shooterFollowerEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Shooter/Shooter Follower Motor Temp", _shooterMotorFollowerSpin.GetMotorTemperature());
    frc::SmartDashboard::PutNumber("Shooter/Shooter Main Motor Temp", _shooterMotorMainSpin.GetMotorTemperature());
}   

frc2::CommandPtr SubShooter::ShootNote(){
    return StartEnd(
    [this]{_shooterMotorMainSpin.Set(0.6), _shooterMotorFollowerSpin.Set(0.9);},
    [this]{_shooterMotorMainSpin.Set(0), _shooterMotorFollowerSpin.Set(0);}
    );
}
