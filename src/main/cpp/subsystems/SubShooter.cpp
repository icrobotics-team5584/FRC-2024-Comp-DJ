// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubShooter.h"
#include <frc/smartdashboard/SmartDashboard.h>

SubShooter::SubShooter(){
    _secondaryShooterMotorSpin.RestoreFactoryDefaults();
    _shooterMotorMainSpin.RestoreFactoryDefaults();

    _secondaryShooterMotorSpin.SetInverted(true);

    frc::SmartDashboard::PutNumber("Main Shooter/ 1 Request Shooter Power Main: ", mainMotorPower);
    frc::SmartDashboard::PutNumber("Secondary Shooter/ 1 Request Secondary Shooter Power: ", secondaryMotorPower);
}

using namespace frc2::cmd;

// This method will be called once per scheduler run
void SubShooter::Periodic() {
    mainMotorPower = frc::SmartDashboard::GetNumber("Main Shooter/ 1 Request Shooter Power Main: ", 0);
    frc::SmartDashboard::PutNumber("Main Shooter/ 2 Main Motor velocity: ", _shooterMainEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Main Shooter/ 3 Main Shooter Motor Power: ", _shooterMotorMainSpin.Get());
    frc::SmartDashboard::PutNumber("Main Shooter/ 4 Main Motor Temp: ", _shooterMotorMainSpin.GetMotorTemperature());

    secondaryMotorPower = frc::SmartDashboard::GetNumber("Secondary Shooter/ 1 Request Secondary Shooter Power: ", 0);  
    frc::SmartDashboard::PutNumber("Secondary Shooter/ 2 Secondary Shooter Motor Velocity: ", _secondaryShooterEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Secondary Shooter/ 3 Secondary Shooter Motor Power: ", _secondaryShooterMotorSpin.Get());
    frc::SmartDashboard::PutNumber("Secondary Shooter/ 4 Secondary Shooter Motor Temp: ", _secondaryShooterMotorSpin.GetMotorTemperature());
    
}   

frc2::CommandPtr SubShooter::ShootNote(){
    return StartEnd(
        [this]{_shooterMotorMainSpin.Set(mainMotorPower), _secondaryShooterMotorSpin.Set(secondaryMotorPower);},
        [this]{_shooterMotorMainSpin.Set(0), _secondaryShooterMotorSpin.Set(0);}
    );
}
