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
#include "subsystems/SubClimber.h"
#include <frc2/command/Commands.h>
#include "commands/UniversalCommands.h"
#include "subsystems/SubVision.h"
#include "commands/VisionCommands.h"
#include "subsystems/SubArm.h"
#include "utilities/POVHelper.h"

RobotContainer::RobotContainer() {
  pathplanner::NamedCommands::registerCommand("Intake", SubIntake::GetInstance().Intake());
  pathplanner::NamedCommands::registerCommand("StopIntakeSpinning", SubIntake::GetInstance().StopSpinningIntake());
  pathplanner::NamedCommands::registerCommand("StartShooter", SubShooter::GetInstance().StartShooter());
  pathplanner::NamedCommands::registerCommand("RetractIntake", SubIntake::GetInstance().CommandRetractIntake());
  pathplanner::NamedCommands::registerCommand("ShootNote", SubShooter::GetInstance().ShootSequence());
  pathplanner::NamedCommands::registerCommand("StopShooter", SubShooter::GetInstance().StopShooterCommand());
  pathplanner::NamedCommands::registerCommand("FeedNote", SubArm::GetInstance().FeedNote());
  pathplanner::NamedCommands::registerCommand("Shoot3_s", cmd::ShootFullSequence().WithTimeout(3_s));

  
  SubArm::GetInstance();
  SubDrivebase::GetInstance();
  SubIntake::GetInstance();
  SubVision::GetInstance();

  SubDrivebase::GetInstance().SetDefaultCommand(SubDrivebase::GetInstance().JoystickDrive(_driverController));
  SubClimber::GetInstance().SetDefaultCommand(SubClimber::GetInstance().JoyStickDrive(_operatorController));
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
  _autoChooser.AddOption("A4", "A4");
  _autoChooser.AddOption("Alliance collect path", "Alliance collect path");
  frc::SmartDashboard::PutData("Chosen Path", &_autoChooser);

  _compressor.EnableAnalog(70_psi, 120_psi);
}

void RobotContainer::ConfigureBindings() {

  // use for robot testing

  _driverController.Start().OnTrue(SubDrivebase::GetInstance().ResetGyroCmd()); //working

  _driverController.LeftTrigger().OnTrue(SubClimber::GetInstance().ClimberManualDrive(0.5));
  _driverController.LeftTrigger().OnFalse(SubClimber::GetInstance().ClimberManualDrive(0));
  _driverController.RightTrigger().OnTrue(SubClimber::GetInstance().ClimberManualDrive(-0.5));
  _driverController.RightTrigger().OnFalse(SubClimber::GetInstance().ClimberManualDrive(0));

  _driverController.A().OnTrue(SubClimber::GetInstance().ClimberAutoReset());
  _driverController.B().OnTrue(SubClimber::GetInstance().ClimberPosition(0.625_m));
  _driverController.X().OnTrue(SubClimber::GetInstance().ClimberPosition(0.4_m));
  _driverController.Y().OnTrue(SubClimber::GetInstance().ClimberPosition(0.02_m));

  _operatorController.A().OnTrue(SubIntake::GetInstance().ExtendIntake());

  // _driverController.Y().OnTrue(frc2::cmd::RunOnce( [] { SubDrivebase::GetInstance().ResetGyroHeading(); } ));

  _operatorController.X().OnTrue(SubClimber::GetInstance().ClimberExtend());    // working
  _operatorController.Y().OnTrue(SubClimber::GetInstance().ClimberRetract());   // working
  _operatorController.A().WhileTrue(SubShooter::GetInstance().StartShooter());  // working
  _operatorController.RightTrigger().WhileTrue(cmd::ShootFullSequence());       // working
  _operatorController.LeftBumper().OnFalse(SubShooter::GetInstance().ShooterChangePosClose());  // working
  _operatorController.RightBumper().OnFalse(SubShooter::GetInstance().ShooterChangePosFar());   // working
  _operatorController.LeftTrigger().WhileTrue(cmd::IntakefullSequence());  // working
  _operatorController.Start().WhileTrue(SubClimber::GetInstance().ClimberAutoReset());
  POVHelper::Up(&_operatorController).OnTrue(SubClimber::GetInstance().ClimberManualDrive(0.5));
  POVHelper::Up(&_operatorController).OnFalse(SubClimber::GetInstance().ClimberManualDrive(0));
  POVHelper::Down(&_operatorController).OnTrue(SubClimber::GetInstance().ClimberManualDrive(0.5));
  POVHelper::Down(&_operatorController).OnFalse(SubClimber::GetInstance().ClimberManualDrive(0));

  // new controls below WIP 
  /*
  _driverController.Start().OnTrue(SubDrivebase::GetInstance().ResetGyroCmd());
  _driverController.Y().OnTrue(frc2::cmd::RunOnce( [] { SubDrivebase::GetInstance().ResetGyroHeading(); } ));

  _operatorController.Start().OnTrue(nullptr outtake/eject);

  _operatorController.LeftTrigger().WhileTrue(cmd::IntakefullSequence());
  _operatorController.LeftBumper().OnTrue(SubShooter::GetInstance().ShooterChangePosClose());
  _operatorController.RightBumper().OnTrue(SubShooter::GetInstance().ShooterChangePosFar());
  _operatorController.RightTrigger().WhileTrue(cmd::ShootFullSequence());

  _operatorController.Y().OnTrue(cmd::TrapSequence());
  _operatorController.X().OnTrue(nullptr climb sequence);
  _operatorController.A().OnTrue(cmd::ArmToAmpPos());
  _operatorController.B().OnTrue(SubShooter::GetInstance().StartShooter());

  //_operatorController.POVLeft(true).OnTrue(SubLED::GetInstance().IndicateSourceDrop()); */

  //Operator controls sysID
  // _operatorController.A().WhileTrue(SubArm::GetInstance().SysIdDynamic(frc2::sysid::Direction::kForward));
  // _operatorController.B().WhileTrue(SubArm::GetInstance().SysIdDynamic(frc2::sysid::Direction::kReverse));
  // _operatorController.X().WhileTrue(SubArm::GetInstance().SysIdQuasistatic(frc2::sysid::Direction::kForward));
  // _operatorController.Y().WhileTrue(SubArm::GetInstance().SysIdQuasistatic(frc2::sysid::Direction::kReverse));

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  _autoSelected = _autoChooser.GetSelected();
  units::second_t delay = _delayChooser.GetSelected() * 1_s;
  return frc2::cmd::Wait(delay).AndThen(pathplanner::PathPlannerAuto(_autoSelected).ToPtr()); 
}

