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
#include <frc2/command/button/CommandXboxController.h>
#include "commands/UniversalCommands.h"
#include "utilities/BotVars.h"
#include "subsystems/SubVision.h"
#include "commands/VisionCommands.h"
#include "subsystems/SubArm.h"
#include "utilities/POVHelper.h"

RobotContainer::RobotContainer() {
  frc::SmartDashboard::PutData("Command Scheduler", &frc2::CommandScheduler::GetInstance());

  pathplanner::NamedCommands::registerCommand("Intake", SubIntake::GetInstance().Intake());
  pathplanner::NamedCommands::registerCommand("StopIntakeSpinning",
                                              SubIntake::GetInstance().StopSpinningIntake());
  pathplanner::NamedCommands::registerCommand("StartShooter",
                                              SubShooter::GetInstance().StartShooter());
  pathplanner::NamedCommands::registerCommand("RetractInt ake",
                                              SubIntake::GetInstance().CommandRetractIntake());
  pathplanner::NamedCommands::registerCommand("ShootNote",  
    SubShooter::GetInstance().ShootSequence());
  pathplanner::NamedCommands::registerCommand("StopShooter",
                                              SubShooter::GetInstance().StopShooterCommand());
  pathplanner::NamedCommands::registerCommand("FeedNote", SubArm::GetInstance().FeedNote());
  // pathplanner::NamedCommands::registerCommand("ShootFullSequence",
  // cmd::ShootFullSequence().WithTimeout(0.5_s));
  pathplanner::NamedCommands::registerCommand("AutoShootFullSequence",
                                              cmd::AutoShootFullSequence().WithTimeout(0.5_s));
  pathplanner::NamedCommands::registerCommand("StoreNote", SubArm::GetInstance().StoreNote());
  pathplanner::NamedCommands::registerCommand("ShooterChangePosFar",
                                              SubShooter::GetInstance().ShooterChangePosFar());
  pathplanner::NamedCommands::registerCommand("ShooterChangePosClose",
                                              SubShooter::GetInstance().ShooterChangePosClose());
  pathplanner::NamedCommands::registerCommand("StopFeeder", SubShooter::GetInstance().StopFeeder());
  // pathplanner::NamedCommands::registerCommand("Shoot3_s",
  // cmd::ShootFullSequenceWithVision(controller).WithTimeout(3_s));

  SubArm::GetInstance();
  SubDrivebase::GetInstance();
  SubIntake::GetInstance();
  SubVision::GetInstance();

  SubDrivebase::GetInstance().SetDefaultCommand(
      SubDrivebase::GetInstance().JoystickDrive(_driverController, false));
  ConfigureBindings();
  _delayChooser.AddOption("0 Seconds", 0);
  _delayChooser.AddOption("1 Seconds", 1);
  _delayChooser.AddOption("2 Seconds", 2);
  frc::SmartDashboard::PutData("Delay By", &_delayChooser);

  _autoChooser.AddOption("A10", "A10");
  _autoChooser.AddOption("Test Path", "Test Path");
  _autoChooser.AddOption("M4", "M4");
  _autoChooser.AddOption("S1", "S1");
  _autoChooser.AddOption("Alliance collect path", "Alliance collect path");
  _autoChooser.AddOption("Nothing", "Nothing");
  frc::SmartDashboard::PutData("Chosen Path", &_autoChooser);

  _compressor.EnableAnalog(70_psi, 120_psi);
}

void RobotContainer::ConfigureBindings() {
  // SOFTWARE CONTROLS

  //   _driverController.Start().OnTrue(SubDrivebase::GetInstance().ResetGyroCmd()); //working

  //   _driverController.LeftBumper().WhileTrue(cmd::ArmToAmpPos()); //working
  //   _driverController.LeftBumper().OnFalse(cmd::ArmToStow()); //working
  //   _driverController.LeftTrigger().WhileTrue(cmd::IntakefullSequence());
  // //  _driverController.B().OnTrue(SubIntake::GetInstance().ExtendIntake());
  //   _driverController.X().WhileTrue(cmd::OuttakeNote());

  //   _driverController.RightBumper().WhileTrue(cmd::ShootFullSequenceWithoutVision());
  //   _driverController.RightTrigger().WhileTrue(cmd::ShootFullSequenceWithVision());

  //   _driverController.A().OnTrue(cmd::VisionRotateToZero());
  //   _driverController.B().OnTrue(SubIntake::GetInstance().ExtendIntake());
  //   _driverController.Y().OnTrue(cmd::PrepareToShoot());
  //   //POVHelper::Up(&_driverController).WhileTrue(SubShooter::GetInstance().StartFeeder());

  //   _operatorController.RightTrigger().WhileTrue(cmd::ShootFullSequenceWithVision());
  //   _operatorController.RightBumper().OnFalse(SubShooter::GetInstance().ShooterChangePosFar());

  //   _operatorController.LeftBumper().OnFalse(SubShooter::GetInstance().ShooterChangePosClose());
  //   _operatorController.LeftTrigger().WhileTrue(cmd::IntakefullSequence());

  //   _operatorController.X().OnTrue(SubClimber::GetInstance().ClimberPosition(0.625_m));
  //   _operatorController.Y().OnTrue(SubClimber::GetInstance().ClimberPosition(0.02_m));
  //   _operatorController.A().WhileTrue(SubShooter::GetInstance().StartShooter());

  //   _operatorController.Start().WhileTrue(SubClimber::GetInstance().ClimberAutoReset());

  //   POVHelper::Up(&_operatorController).OnTrue(SubClimber::GetInstance().ClimberManualDrive(0.5));
  //   POVHelper::Up(&_operatorController).OnFalse(SubClimber::GetInstance().ClimberManualDrive(0));
  //   POVHelper::Down(&_operatorController).OnTrue(SubClimber::GetInstance().ClimberManualDrive(0.5));
  //   POVHelper::Down(&_operatorController).OnFalse(SubClimber::GetInstance().ClimberManualDrive(0));

  // DRIVER CONTROLS V6

  _driverController.Y().OnTrue(SubDrivebase::GetInstance().ResetGyroCmd());
  // _driverController.LeftTrigger().WhileTrue(/*Align2Stage*/);
  _driverController.LeftBumper().WhileTrue(cmd::IntakeFromSource());
  _driverController.RightTrigger().WhileTrue(cmd::VisionRotateToSpeaker(_driverController));
  _driverController.LeftTrigger().WhileTrue(cmd::VisionClimb());
  // _driverController.RightBumper().WhileTrue(/*Align2Amp*/);
  _driverController.A().WhileTrue(SubClimber::GetInstance().ClimberAutoReset());

  _operatorController.Start().WhileTrue(cmd::OuttakeNote());
  _operatorController.Back().WhileTrue(cmd::OuttakeIntakeAndEndEffector());

  _operatorController.LeftTrigger().WhileTrue(cmd::IntakefullSequence());
  _operatorController.LeftBumper().OnTrue(SubShooter::GetInstance().ShooterChangePosClose());
  _operatorController.RightBumper().OnTrue(SubShooter::GetInstance().ShooterChangePosFar());
  _operatorController.RightTrigger().WhileTrue(cmd::ShootSpeakerOrArm());

  _operatorController.Y().OnTrue(cmd::ArmToTrapPos());
  _operatorController.Y().OnFalse(cmd::ArmToStow());
  _operatorController.X().WhileTrue(cmd::ShootIntoAmp());
  _operatorController.B().OnTrue(cmd::ArmToAmpPos());
  _operatorController.B().OnFalse(cmd::ArmToStow());
  _operatorController.A().OnTrue(cmd::PrepareToShoot());

  POVHelper::Up(&_operatorController).OnTrue(SubClimber::GetInstance().ClimberPosition(0.498_m));
  POVHelper::Down(&_operatorController).OnTrue(SubClimber::GetInstance().ClimberPosition(0.02_m));
  POVHelper::Left(&_operatorController).OnTrue(SubIntake::GetInstance().ExtendIntake());
  POVHelper::Right(&_operatorController).OnTrue(SubClimber::GetInstance().ClimberPosition(0.35_m));

  frc2::Trigger(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop(), [=, this] {
    return (_operatorController.GetLeftY() < -0.2 || _operatorController.GetLeftY() > 0.2) &&
    !(_operatorController.GetRightY() < -0.2 || _operatorController.GetRightY() > 0.2);
  }).WhileTrue(SubClimber::GetInstance().ClimberJoystickDriveLeft(_operatorController));

  frc2::Trigger(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop(), [=, this] {
    return (_operatorController.GetRightY() < -0.2 || _operatorController.GetRightY() > 0.2) &&
    !(_operatorController.GetLeftY() < -0.2 || _operatorController.GetLeftY() > 0.2);
  }).WhileTrue(SubClimber::GetInstance().ClimberJoystickDriveRight(_operatorController));

  frc2::Trigger(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop(), [=, this] {
    return (_operatorController.GetRightY() < -0.2 || _operatorController.GetRightY() > 0.2) &&
           (_operatorController.GetLeftY() < -0.2 || _operatorController.GetLeftY() > 0.2);
  }).WhileTrue(SubClimber::GetInstance().ClimberJoystickDrive(_operatorController));

  // Operator controls sysID
  // _operatorController.A().WhileTrue(SubArm::GetInstance().SysIdDynamic(frc2::sysid::Direction::kForward));
  // _operatorController.B().WhileTrue(SubArm::GetInstance().SysIdDynamic(frc2::sysid::Direction::kReverse));
  // _operatorController.X().WhileTrue(SubArm::GetInstance().SysIdQuasistatic(frc2::sysid::Direction::kForward));
  // _operatorController.Y().WhileTrue(SubArm::GetInstance().SysIdQuasistatic(frc2::sysid::Direction::kReverse));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  _autoSelected = _autoChooser.GetSelected();
  units::second_t delay = _delayChooser.GetSelected() * 1_s;
  return frc2::cmd::Wait(delay)
      .AndThen(pathplanner::PathPlannerAuto(_autoSelected).ToPtr())
      .AlongWith(SubClimber::GetInstance().ClimberAutoReset().AndThen(
          SubClimber::GetInstance().ClimberPosition(0.35_m)));
}
