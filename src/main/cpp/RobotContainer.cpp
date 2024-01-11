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

  // Amp Shooter
  _driverController.LeftTrigger().WhileTrue(SubAmp::GetInstance().AmpShooter());
  _driverController.RightTrigger().WhileTrue(SubAmp::GetInstance().ReverseAmpShooter());

  // Arm
  _driverController.A().WhileTrue(SubAmp::GetInstance().TiltArmToAngle(0_deg));
  _driverController.B().WhileTrue(SubAmp::GetInstance().TiltArmToAngle(180_deg));
  _driverController.X().WhileTrue(SubAmp::GetInstance().TiltArmToAngle(20_deg)); 
  _driverController.Y().WhileTrue(SubAmp::GetInstance().TiltArmToAngle(40_deg));
  }

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
