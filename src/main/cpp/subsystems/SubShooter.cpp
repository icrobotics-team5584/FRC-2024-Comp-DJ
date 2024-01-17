// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubShooter.h"
#include <frc/smartdashboard/SmartDashboard.h>

SubShooter::SubShooter(){
    _secondaryShooterMotorSpin.RestoreFactoryDefaults();
    _shooterMotorMainSpin.RestoreFactoryDefaults();

    _secondaryShooterMotorSpin.SetInverted(true);
    frc::SmartDashboard::PutString("Shooter/Shooter Angle: ", "Score From Podium");
}

using namespace frc2::cmd;

// This method will be called once per scheduler run
void SubShooter::Periodic() {
    frc::SmartDashboard::PutNumber("Main Shooter/ 2 Main Motor velocity: ", _shooterMainEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Main Shooter/ 3 Main Shooter Motor Power: ", _shooterMotorMainSpin.Get());
    frc::SmartDashboard::PutNumber("Main Shooter/ 4 Main Motor Temp: ", _shooterMotorMainSpin.GetMotorTemperature());

    frc::SmartDashboard::PutNumber("Secondary Shooter/ 2 Secondary Shooter Motor Velocity: ", _secondaryShooterEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Secondary Shooter/ 3 Secondary Shooter Motor Power: ", _secondaryShooterMotorSpin.Get());
    frc::SmartDashboard::PutNumber("Secondary Shooter/ 4 Secondary Shooter Motor Temp: ", _secondaryShooterMotorSpin.GetMotorTemperature());
    frc::SmartDashboard::PutNumber("Shooter/Shooter Piston Position", solShooter.Get());

    if (solShooter.Get() == frc::DoubleSolenoid::kReverse){
        frc::SmartDashboard::PutString("Shooter/Shooter Angle: ", "Score From Podium");
    } else{frc::SmartDashboard::PutString("Shooter/Shooter Angle: ", "Score From Subwoofer");}
    
}   


frc2::CommandPtr SubShooter::ShootNoteFixed(){
    return StartEnd(
        [this]{
            if(solShooter.Get() == frc::DoubleSolenoid::kReverse){_shooterMotorMainSpin.Set(0.6), _secondaryShooterMotorSpin.Set(0.6);} 
            else {_shooterMotorMainSpin.Set(0.4), _secondaryShooterMotorSpin.Set(0.4);}},
        [this]{_shooterMotorMainSpin.Set(0), _secondaryShooterMotorSpin.Set(0);}
    );
}

frc2::CommandPtr SubShooter::ChangeAngle(){
    return RunOnce(
        [this]{
            if(solShooter.Get() == frc::DoubleSolenoid::kReverse){
                solShooter.Set(frc::DoubleSolenoid::kForward);
            } else {solShooter.Set(frc::DoubleSolenoid::kReverse);}
        }
    );
}

