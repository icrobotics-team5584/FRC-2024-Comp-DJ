// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <frc2/command/Commands.h>
#include "subsystems/SubShooter.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <subsystems/SubDrivebase.h>

#include "RobotContainer.h"
#include "subsystems/SubAmp.h"
#include "subsystems/SubClimber.h"
#include "commands/ClimberCommands.h"
#include <frc2/command/Commands.h>

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
    _driverController.Start().OnTrue(SubDrivebase::GetInstance().SyncSensorBut());
    _driverController.Back().OnTrue(SubDrivebase::GetInstance().ResetGyroCmd());
    _driverController.LeftTrigger().WhileTrue(SubAmp::GetInstance().AmpShooter());
    _driverController.RightTrigger().WhileTrue(SubAmp::GetInstance().ReverseAmpShooter());

  // Arm
    _driverController.A().WhileTrue(SubAmp::GetInstance().TiltArmToAngle(0_deg));
    _driverController.B().WhileTrue(SubAmp::GetInstance().TiltArmToAngle(180_deg));
    _driverController.X().WhileTrue(SubAmp::GetInstance().TiltArmToAngle(20_deg)); 
    _driverController.Y().WhileTrue(SubAmp::GetInstance().TiltArmToAngle(40_deg));
    _driverController.X().OnTrue(SubDrivebase::GetInstance().SyncSensorBut());
    _driverController.Y().OnTrue(SubDrivebase::GetInstance().ResetGyroCmd());
    _driverController.RightBumper().OnTrue(SubShooter::GetInstance().ChangeAngle());
    _driverController.RightTrigger().OnTrue(SubShooter::GetInstance().StartShooter());
    _driverController.LeftBumper().OnTrue(SubShooter::GetInstance().ShootSequence());
    _driverController.A().OnTrue(cmd::ClimberExtend());
    _driverController.B().OnTrue(cmd::ClimberRetract());

  //Use below if above don't work
  // _driverController.A().OnTrue(cmd::ClimberExtendManual());
  // _driverController.B().OnTrue(cmd::ClimberRetractManual());
  _driverController.X().OnTrue(cmd::ClimberStop());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
    _autoSelected = _autoChooser.GetSelected();
    units::second_t delay = _delayChooser.GetSelected() *1_s;
  return frc2::cmd::Wait(delay).AndThen(pathplanner::PathPlannerAuto(_autoSelected).ToPtr());
}
