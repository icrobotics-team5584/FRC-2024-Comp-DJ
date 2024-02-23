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
#include "subsystems/SubArm.h"

RobotContainer::RobotContainer() {
  pathplanner::NamedCommands::registerCommand("ExtendIntake", SubIntake::GetInstance().ExtendIntake());
  pathplanner::NamedCommands::registerCommand("StartIntakeSpinning", SubIntake::GetInstance().StartSpinningIntake());
  pathplanner::NamedCommands::registerCommand("StopIntakeSpinning", SubIntake::GetInstance().StopSpinningIntake());
  pathplanner::NamedCommands::registerCommand("StartShooter", SubShooter::GetInstance().StartShooter());
  pathplanner::NamedCommands::registerCommand("RetractIntake", SubIntake::GetInstance().CommandRetractIntake());
  pathplanner::NamedCommands::registerCommand("ShootNote", SubShooter::GetInstance().ShootSequence());
  pathplanner::NamedCommands::registerCommand("StopShooter", SubShooter::GetInstance().StopShooterCommand());

  
  SubArm::GetInstance();
  SubDrivebase::GetInstance();
  SubIntake::GetInstance();
  SubDrivebase::GetInstance().SetDefaultCommand(SubDrivebase::GetInstance().JoystickDrive(_driverController));
  ConfigureBindings();
  _delayChooser.AddOption("0 Seconds", 0);
  _delayChooser.AddOption("1 Seconds", 1);
  _delayChooser.AddOption("2 Seconds", 2);
  frc::SmartDashboard::PutData("Delay By", &_delayChooser);

  _compressor.EnableAnalog(70_psi, 120_psi);
}

void RobotContainer::ConfigureBindings() {
  _driverController.Start().OnTrue(SubDrivebase::GetInstance().ResetGyroCmd()); //working

  _driverController.LeftTrigger().WhileTrue(cmd::ShootFullSequence()); //working
  _driverController.RightTrigger().WhileTrue(cmd::IntakefullSequence());//working     /*.AndThen([this]{_driverController.SetRumble(frc::GenericHID::kBothRumble, 1); _operatorController.SetRumble(frc::GenericHID::kBothRumble, 1);})*/)

  _driverController.LeftBumper().OnTrue(SubArm::GetInstance().TiltArmToAngle(0.25_tr)); //working
  _driverController.LeftBumper().OnFalse(SubArm::GetInstance().TiltArmToAngle(0.125_tr)); //working
  _driverController.RightBumper().OnTrue(SubLED::GetInstance().IndicateAmp()); //working


  _operatorController.B().OnTrue(SubClimber::GetInstance().ClimberExtend()); //working
  _operatorController.A().OnTrue(SubClimber::GetInstance().ClimberRetract()); //working
  _operatorController.X().OnTrue(SubClimber::GetInstance().ClimberLock()); //working
  _operatorController.Y().OnTrue(SubClimber::GetInstance().ClimberUnlock()); //working
  _operatorController.RightTrigger().WhileTrue(SubShooter::GetInstance().StartShooter()); //working
  _operatorController.LeftBumper().OnFalse(SubShooter::GetInstance().ShooterChangePosClose()); //working
  _operatorController.RightBumper().OnFalse(SubShooter::GetInstance().ShooterChangePosFar()); //working
  _operatorController.LeftTrigger().WhileTrue(SubArm::GetInstance().AmpShooter()); //working

  _driverController.Back().OnTrue(SubIntake::GetInstance().ExtendIntake());

   _driverController.A().WhileTrue(SubArm::GetInstance().SysIdDynamic(frc2::sysid::Direction::kForward));
   _driverController.B().WhileTrue(SubArm::GetInstance().SysIdDynamic(frc2::sysid::Direction::kReverse));
   _driverController.X().WhileTrue(SubArm::GetInstance().SysIdQuasistatic(frc2::sysid::Direction::kForward));
   _driverController.Y().WhileTrue(SubArm::GetInstance().SysIdQuasistatic(frc2::sysid::Direction::kReverse));


}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  _autoSelected = _autoChooser.GetSelected();
  units::second_t delay = _delayChooser.GetSelected() * 1_s;
  return frc2::cmd::Wait(delay).AndThen(pathplanner::PathPlannerAuto(_autoSelected).ToPtr());
}

