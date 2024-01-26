// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubIntake.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc2::cmd;

SubIntake::SubIntake() {
  frc::SmartDashboard::PutString("Intake/Intake Deploy State: ", "Intake retracted");
  _intakeMotorSpin.RestoreFactoryDefaults();
  _intakeMotorSpin.SetSmartCurrentLimit(30);
}
// This method will be called once per scheduler run
void SubIntake::Periodic() {
  if (solIntake.Get() == 1) {
    frc::SmartDashboard::PutString("Intake/Intake Deploy State: ", "Intake deployed");
  } else if (solIntake.Get() == 2) {
    frc::SmartDashboard::PutString("Intake/Intake Deploy State: ", "Intake retracted");
  }
  frc::SmartDashboard::PutNumber("Intake/Intake Shooter Motor: ", _intakeMotorSpin.Get());
}

frc2::CommandPtr SubIntake::ExtendIntake() {
  return Run([this] { solIntake.Set(frc::DoubleSolenoid::kForward); }).Until([this] {
    return GetIntakeState();
  });
}

frc2::CommandPtr SubIntake::RetractIntake() {
  return Run([this] { solIntake.Set(frc::DoubleSolenoid::kReverse); }).Until([this] {
    return GetIntakeState();
  });
}

frc2::CommandPtr SubIntake::StopSpinningIntake() {
  return RunOnce([this] { _intakeMotorSpin.Set(0); });
}

frc2::CommandPtr SubIntake::StartSpinningIntake() {
  return RunOnce([this] { _intakeMotorSpin.Set(0.5); });
}

bool SubIntake::GetIntakeState() {
  if (_intakeExtendededReed.Get() == true || _intakeRetractedReed.Get() == true) {
    return true;
  } else {
    return false;
  }
}  // LOCK ARM IF RETURN FALSE