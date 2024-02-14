// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubShooter.h"
#include <frc/smartdashboard/SmartDashboard.h>

SubShooter::SubShooter() {
  _secondaryShooterMotorSpin.RestoreFactoryDefaults();
  _shooterMotorMainSpin.RestoreFactoryDefaults();
  
  solShooter.Set(frc::DoubleSolenoid::kReverse);

  _secondaryShooterMotorSpin.SetInverted(true);
  _shooterMotorMainSpin.SetPIDFF(ShooterP, ShooterI, ShooterD, ShooterFF);
  _secondaryShooterMotorSpin.SetPIDFF(ShooterP, ShooterI, ShooterD, ShooterFF);
  frc::SmartDashboard::PutData("Shooter/Main Motor", (wpi::Sendable*)&_shooterMotorMainSpin);
  frc::SmartDashboard::PutData("Shooter/Second Motor", (wpi::Sendable*)&_secondaryShooterMotorSpin);
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
          _shooterMotorMainSpin.SetVelocityTarget(ShootFarTargetRPM*1_rpm);
              _secondaryShooterMotorSpin.SetVelocityTarget(ShootFarTargetRPM*1_rpm);
        } else {
          _shooterMotorMainSpin.SetVelocityTarget(ShootCloseTargetRPM*1_rpm);
              _secondaryShooterMotorSpin.SetVelocityTarget(ShootCloseTargetRPM*1_rpm);
        }
      },
      [this] { _shooterMotorMainSpin.Set(0), _secondaryShooterMotorSpin.Set(0); });
}

frc2::CommandPtr SubShooter::ShootNote() {
  return RunOnce([this] {
    if (solShooter.Get() == frc::DoubleSolenoid::kReverse) {
      if (_shooterMotorMainSpin.GetVelocity() >= 3400_rpm &&
          _secondaryShooterMotorSpin.GetVelocity() >= 3400_rpm) {
        _shooterFeederMotor.Set(0.5);
      }
    } else if (solShooter.Get() == frc::DoubleSolenoid::kForward) {
      if (_shooterMotorMainSpin.GetVelocity() >= 2200_rpm &&
          _secondaryShooterMotorSpin.GetVelocity() >= 2200_rpm) {
        _shooterFeederMotor.Set(0.5);
      }
    }
  });
}

frc2::CommandPtr SubShooter::ShootSequence() {
  return Sequence(StartShooter(), ShootNote());
}

frc2::CommandPtr SubShooter::ShooterChangePosFar() {
  return RunOnce([this] { solShooter.Set(frc::DoubleSolenoid::kReverse); });
}

frc2::CommandPtr SubShooter::ShooterChangePosClose() {
  return RunOnce([this] { solShooter.Set(frc::DoubleSolenoid::kForward); });
}