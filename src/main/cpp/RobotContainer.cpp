// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <frc2/command/Commands.h>
#include "subsystems/SubShooter.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <subsystems/SubDrivebase.h>
#include "subsystems/SubIntake.h"
#include "subsystems/SubLED.h"
#include <pathplanner/lib/auto/NamedCommands.h>
#include "RobotContainer.h"
#include "subsystems/SubAmp.h"
#include "subsystems/SubClimber.h"
#include <frc2/command/Commands.h>
#include "commands/UniversalCommands.h"

RobotContainer::RobotContainer() {
  pathplanner::NamedCommands::registerCommand("ExtendIntake", SubIntake::GetInstance().ExtendIntake());
  pathplanner::NamedCommands::registerCommand("StartIntakeSpinning", SubIntake::GetInstance().StartSpinningIntake());
  pathplanner::NamedCommands::registerCommand("StopIntakeSpinning", SubIntake::GetInstance().StopSpinningIntake());
  pathplanner::NamedCommands::registerCommand("StartShooter", SubShooter::GetInstance().StartShooter());
  pathplanner::NamedCommands::registerCommand("RetractIntake", SubIntake::GetInstance().CommandRetractIntake());
  pathplanner::NamedCommands::registerCommand("ShootNote", SubShooter::GetInstance().ShootSequence());
  pathplanner::NamedCommands::registerCommand("StopShooter", SubShooter::GetInstance().StopShooterCommand());

  
  SubAmp::GetInstance();
  SubDrivebase::GetInstance();
  SubIntake::GetInstance();
  SubDrivebase::GetInstance().SetDefaultCommand(SubDrivebase::GetInstance().JoystickDrive(_driverController));
  ConfigureBindings();
  _delayChooser.AddOption("0 Seconds", 0);
  _delayChooser.AddOption("1 Seconds", 1);
  _delayChooser.AddOption("2 Seconds", 2);
  frc::SmartDashboard::PutData("Delay By", &_delayChooser);

  _autoChooser.AddOption("Middle Path", "Middle Path");
  _autoChooser.AddOption("Amp Path", "Amp Path");
  _autoChooser.AddOption("Podium Path", "Podium Path");
  _autoChooser.AddOption("Mid Path-Break Podium", "Mid Path-Break Podium");
  _autoChooser.AddOption("Mid Path-Break Amp", "Mid Path-Break Amp");
  _autoChooser.AddOption("Test Path", "Test Path");
  _autoChooser.AddOption("Alliance collect path", "Alliance collect path");
frc::SmartDashboard::PutData("Chosen Path", &_autoChooser);
  }

void RobotContainer::ConfigureBindings() {
  _driverController.Start().OnTrue(SubDrivebase::GetInstance().ResetGyroCmd()); //working

  // _driverController.LeftTrigger().WhileTrue(cmd::ShootSequence());
  // _driverController.RightTrigger().WhileTrue(cmd::IntakeSequence().AndThen([this]{_driverController.SetRumble(frc::GenericHID::kBothRumble, 1); _operatorController.SetRumble(frc::GenericHID::kBothRumble, 1);}));

  _driverController.A().OnTrue(SubClimber::GetInstance().ClimberManualDrive(1));
  _driverController.B().OnTrue(SubClimber::GetInstance().ClimberManualDrive(-1));
  _driverController.X().OnTrue(SubClimber::GetInstance().ClimberStop());
  _driverController.Y().OnTrue(SubClimber::GetInstance().ClimberPosition(0.5_m));

  // _driverController.Y().OnTrue(frc2::cmd::RunOnce( [] { SubDrivebase::GetInstance().ResetGyroHeading(); } ));

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  _autoSelected = _autoChooser.GetSelected();
  units::second_t delay = _delayChooser.GetSelected() * 1_s;
  return frc2::cmd::Wait(delay).AndThen(pathplanner::PathPlannerAuto(_autoSelected).ToPtr());
}
