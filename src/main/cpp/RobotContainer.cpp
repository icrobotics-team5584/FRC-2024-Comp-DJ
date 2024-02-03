// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <subsystems/SubDrivebase.h>

#include "RobotContainer.h"
#include "subsystems/SubArm.h"

RobotContainer::RobotContainer() {
  SubArm::GetInstance();
SubDrivebase::GetInstance();
SubDrivebase::GetInstance().SetDefaultCommand(SubDrivebase::GetInstance().JoystickDrive(_driverController));
  ConfigureBindings();
  _delayChooser.AddOption("0 Seconds", 0);
  _delayChooser.AddOption("1 Seconds", 1);
  _delayChooser.AddOption("2 Seconds", 2);
  frc::SmartDashboard::PutData("Delay By", &_delayChooser);

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
    _driverController.LeftTrigger().WhileTrue(SubArm::GetInstance().AmpShooter());
    _driverController.RightTrigger().WhileTrue(SubArm::GetInstance().ReverseAmpShooter());

  // Arm
    _driverController.A().WhileTrue(SubArm::GetInstance().TiltArmToAngle(0_deg));
    _driverController.B().WhileTrue(SubArm::GetInstance().TiltArmToAngle(180_deg));
    _driverController.X().WhileTrue(SubArm::GetInstance().TiltArmToAngle(20_deg)); 
    _driverController.Y().WhileTrue(SubArm::GetInstance().TiltArmToAngle(40_deg));
}
using namespace frc2::cmd;

  //Main Controller
 // _driverController.Start().OnTrue(frc2::cmd::RunOnce([]{SubDriveBase::GetInstance().ResetGyroHeading();}));
 // _driverController.leftJoystick().WhileTrue(frc2::cmd::RunEnd([]{SubDrivebase::GetInstance().Drive();}));





frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
    _autoSelected = _autoChooser.GetSelected();
    units::second_t delay = _delayChooser.GetSelected() *1_s;
  return frc2::cmd::Wait(delay).AndThen(pathplanner::PathPlannerAuto(_autoSelected).ToPtr());
}

};
