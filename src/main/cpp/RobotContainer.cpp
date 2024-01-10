// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/Commands.h>

#include "RobotContainer.h"
#include "subsystems/SubAmp.h"

RobotContainer::RobotContainer() {
  SubAmp::GetInstance();
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  using namespace frc2::cmd;

  // Motor to angle auto
  _driverController.Start().WhileTrue(SubAmp::GetInstance().MotorTiltToAngle());

  // Amp Shooter
  _driverController.A().WhileTrue(SubAmp::GetInstance().AmpShooter());
  _driverController.B().WhileTrue(SubAmp::GetInstance().ReverseAmpShooter());

  // Dizzy Amp
  _driverController.LeftTrigger().WhileTrue(SubAmp::GetInstance().ClawTiltDown());
  _driverController.RightTrigger().WhileTrue(SubAmp::GetInstance().ClawTiltUp());
  }

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
