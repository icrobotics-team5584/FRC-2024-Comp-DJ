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

#include "RobotContainer.h"
#include "subsystems/SubAmp.h"
#include "subsystems/SubClimber.h"
#include <frc2/command/Commands.h>
#include "commands/UniversalCommands.h"

RobotContainer::RobotContainer() {
  SubAmp::GetInstance();
  SubDrivebase::GetInstance();
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
  frc::SmartDashboard::PutData("Chosen Path", &_autoChooser);
}

void RobotContainer::ConfigureBindings() {
  _driverController.Start().OnTrue(SubDrivebase::GetInstance().ResetGyroCmd());

  _driverController.LeftTrigger().WhileTrue(cmd::ShootSequence());
  _driverController.RightTrigger().WhileTrue(cmd::IntakeSequence());

  _driverController.LeftBumper().WhileTrue(cmd::SequenceArmToAmpPos());
  _driverController.A().WhileTrue(cmd::SequenceArmToTrapPos());

  _operatorController.Y().OnTrue(SubClimber::GetInstance().ClimberExtend());
  _operatorController.A().OnTrue(SubClimber::GetInstance().ClimberRetract());
  _operatorController.RightTrigger().WhileTrue(SubShooter::GetInstance().StartShooter());
  _operatorController.LeftBumper().OnFalse(SubShooter::GetInstance().ShooterChangePosClose());
  _operatorController.RightBumper().OnFalse(SubShooter::GetInstance().ShooterChangePosFar());
  _operatorController.LeftTrigger().WhileTrue(SubAmp::GetInstance().AmpShooter());

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  _autoSelected = _autoChooser.GetSelected();
  units::second_t delay = _delayChooser.GetSelected() * 1_s;
  return frc2::cmd::Wait(delay).AndThen(pathplanner::PathPlannerAuto(_autoSelected).ToPtr());
}
