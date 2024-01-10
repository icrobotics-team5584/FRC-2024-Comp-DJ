// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/Commands.h>

#include "RobotContainer.h"
#include "subsystems/SubAmp.h"
#include "subsystems/SubClimber.h"
#include "commands/ClimberCommands.h"
#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  SubAmp::GetInstance();
  SubClimber::GetInstance();
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  using namespace frc2::cmd;

  // // Amp Shooter
  // _driverController.Start().WhileTrue(SubAmp::GetInstance().AmpShooter());
  // _driverController.Back().WhileTrue(SubAmp::GetInstance().ReverseAmpShooter());

  // _driverController.LeftBumper().WhileTrue(SubAmp::GetInstance().ExtraMotor());
  // _driverController.RightBumper().WhileTrue(SubAmp::GetInstance().ReverseExtraMotor());


  // // Dizzy Amp
  // _driverController.A().WhileTrue(SubAmp::GetInstance().ClawOpen());
  // _driverController.B().WhileTrue(SubAmp::GetInstance().ClawClose());

  // _driverController.X().WhileTrue(SubAmp::GetInstance().ClawTiltDown());
  // _driverController.Y().WhileTrue(SubAmp::GetInstance().ClawTiltUp());

  //Climber
  _driverController.A().OnTrue(cmd::ClimberUp());
  _driverController.B().OnTrue(cmd::ClimberDown());
  }

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
