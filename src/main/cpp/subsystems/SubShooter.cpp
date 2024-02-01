// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubShooter.h"
#include <frc/smartdashboard/SmartDashboard.h>

SubShooter::SubShooter() {
  _secondaryShooterMotor.RestoreFactoryDefaults();
  _shooterMotorMain.RestoreFactoryDefaults();
  
  solShooter.Set(frc::DoubleSolenoid::kReverse);

  _secondaryShooterMotor.SetInverted(true);
  _shooterMotorMain.SetPIDFF(ShooterP, ShooterI, ShooterD, ShooterFF);
  _secondaryShooterMotor.SetPIDFF(ShooterP, ShooterI, ShooterD, ShooterFF);
  frc::SmartDashboard::PutData("Shooter/Main Motor", (wpi::Sendable*)&_shooterMotorMain);
  frc::SmartDashboard::PutData("Shooter/Second Motor", (wpi::Sendable*)&_secondaryShooterMotor);
}

using namespace frc2::cmd;

// This method will be called once per scheduler run
void SubShooter::Periodic() {
  frc::SmartDashboard::PutNumber("Shooter/Piston Position", solShooter.Get());

  if (solShooter.Get() == frc::DoubleSolenoid::kReverse) {
    frc::SmartDashboard::PutString("Shooter/Angle: ", "Score From Podium");
  } else {
    frc::SmartDashboard::PutString("Shooter/Angle: ", "Score From Subwoofer");
  }
}

frc2::CommandPtr SubShooter::StartShooter() {
  return StartEnd(
      [this] {
        if (solShooter.Get() == frc::DoubleSolenoid::kReverse) {
          _shooterMotorMain.SetVelocityTarget(ShootFarTargetRPM*1_rpm);
              _secondaryShooterMotor.SetVelocityTarget(ShootFarTargetRPM*1_rpm);
        } else {
          _shooterMotorMain.SetVelocityTarget(ShootCloseTargetRPM*1_rpm);
              _secondaryShooterMotor.SetVelocityTarget(ShootCloseTargetRPM*1_rpm);
        }
      },
      [this] { _shooterMotorMain.Set(0), _secondaryShooterMotor.Set(0); });
}

frc2::CommandPtr SubShooter::ShootNote() {
  return RunOnce([this] { _shooterFeederMotor.Set(0.5); })
      .AndThen(Wait(2_s))
      .AndThen([this] { _shooterFeederMotor.Set(0); })
      .OnlyIf([this] { return CheckShooterSpeed(); });
}

frc2::CommandPtr SubShooter::ShootSequence() {
  return Sequence(StartShooter(), ShootNote());
}

bool SubShooter::CheckShooterSpeed(){
 if(_shooterFeederMotor.GetVelError() && _shooterMotorMain.GetVelError() < 30_rpm){
  return true;
 }
}

frc2::CommandPtr SubShooter::ShooterChangePosFar() {
  return RunOnce([this] { solShooter.Set(frc::DoubleSolenoid::kReverse); });
}

frc2::CommandPtr SubShooter::ShooterChangePosClose() {
  return RunOnce([this] { solShooter.Set(frc::DoubleSolenoid::kForward); });
}