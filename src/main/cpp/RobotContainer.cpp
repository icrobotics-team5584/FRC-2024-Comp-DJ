// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/Commands.h>

#include "RobotContainer.h"
#include "subsystems/SubArm.h"

RobotContainer::RobotContainer() {
  SubArm::GetInstance();
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  using namespace frc2::cmd;

  // Amp Shooter
  _driverController.LeftTrigger().WhileTrue(SubArm::GetInstance().AmpShooter());
  _driverController.RightTrigger().WhileTrue(SubArm::GetInstance().ReverseAmpShooter());

  // Arm
  _driverController.A().WhileTrue(SubArm::GetInstance().TiltArmToAngle(0_deg));
  _driverController.B().WhileTrue(SubArm::GetInstance().TiltArmToAngle(180_deg));
  _driverController.X().WhileTrue(SubArm::GetInstance().TiltArmToAngle(20_deg)); 
  _driverController.Y().WhileTrue(SubArm::GetInstance().TiltArmToAngle(40_deg));
  }

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
