// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/Commands.h>

#include "RobotContainer.h"
#include "subsystems/SubAmp.h"
#include "subsystems/SubClimber.h"
#include "commands/ClimberCommands.h"
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <subsystems/SubDrivebase.h>

RobotContainer::RobotContainer() {
  SubAmp::GetInstance();
  SubClimber::GetInstance();
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
    _driverController.X().OnTrue(SubDrivebase::GetInstance().SyncSensorBut());
    _driverController.Y().OnTrue(SubDrivebase::GetInstance().ResetGyroCmd());
    //_driverController.A().WhileTrue(
     //  frc2::cmd::Run(
     //      []{ 
      //         SubDrivebase::GetInstance().Drive(0_mps, 1_mps, 0_deg_per_s, true); 
      //       }
       //  )
   //  );

 // _driverController.A().WhileTrue(SubDrivebase::GetInstance().SysIdDynamic(frc2::sysid::Direction::kForward));
 // _driverController.B().WhileTrue(SubDrivebase::GetInstance().SysIdDynamic(frc2::sysid::Direction::kReverse));
 // _driverController.X().WhileTrue(SubDrivebase::GetInstance().SysIdQuasistatic(frc2::sysid::Direction::kForward));
 // _driverController.Y().WhileTrue(SubDrivebase::GetInstance().SysIdQuasistatic(frc2::sysid::Direction::kReverse));
}
using namespace frc2::cmd;

  //Main Controller
 // _driverController.Start().OnTrue(frc2::cmd::RunOnce([]{SubDriveBase::GetInstance().ResetGyroHeading();}));
 // _driverController.leftJoystick().WhileTrue(frc2::cmd::RunEnd([]{SubDrivebase::GetInstance().Drive();}));




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
