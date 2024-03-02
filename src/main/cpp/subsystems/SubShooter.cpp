// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubShooter.h"
#include <frc/smartdashboard/SmartDashboard.h>

SubShooter::SubShooter() {
  solShooter.Set(frc::DoubleSolenoid::kReverse);

  _secondaryShooterMotor.SetInverted(true);
  frc::SmartDashboard::PutData("Shooter/Main Motor", (wpi::Sendable*)&_shooterMotorMain);
  frc::SmartDashboard::PutData("Shooter/Second Motor", (wpi::Sendable*)&_secondaryShooterMotor);
  frc::SmartDashboard::PutData("Shooter/Feeder motor", (wpi::Sendable*)&_shooterFeederMotor);
  frc::SmartDashboard::PutData("Shooter/Top PID", (wpi::Sendable*)&_topPID);
  frc::SmartDashboard::PutData("Shooter/Bottom PID", (wpi::Sendable*)&_bottomPID);

  _topEncoder.SetSamplesToAverage(7);
  _topEncoder.SetDistancePerPulse(1.00/2048.00);
  _bottomEncoder.SetSamplesToAverage(7);
  _bottomEncoder.SetDistancePerPulse(1.00/2048.00);
 }

using namespace frc2::cmd;

// This method will be called once per scheduler run
void SubShooter::Periodic() {
  frc::SmartDashboard::PutNumber("Shooter/Piston Position", solShooter.Get());

  if (solShooter.Get() == frc::DoubleSolenoid::kReverse) {
    frc::SmartDashboard::PutString("Shooter/Angle: ", "Score From Podium");
  } else {
    frc::SmartDashboard::PutString("Shooter/Angle: ", "Score From Subwoofer");
  }
}

void SubShooter::SimulationPeriodic(){
 _topShooterSim.SetInputVoltage(_shooterMotorMain.GetSimVoltage());
 _topShooterSim.Update(20_ms);
 _shooterMotorMain.UpdateSimEncoder(_topShooterSim.GetAngularPosition(), _topShooterSim.GetAngularVelocity());
 _topEncoderSim.SetDistance(_topShooterSim.GetAngularPosition().convert<units::turns>().value());
 _topEncoderSim.SetRate(_topShooterSim.GetAngularVelocity().convert<units::turns_per_second>().value());

 _bottomShooterSim.SetInputVoltage(_secondaryShooterMotor.GetSimVoltage());
 _bottomShooterSim.Update(20_ms);
 _secondaryShooterMotor.UpdateSimEncoder(_bottomShooterSim.GetAngularPosition(), _bottomShooterSim.GetAngularVelocity());
 _bottomEncoderSim.SetDistance(_bottomShooterSim.GetAngularPosition().convert<units::turns>().value());
 _bottomEncoderSim.SetRate(_bottomShooterSim.GetAngularVelocity().convert<units::turns_per_second>().value());
 
 _feederSim.SetInputVoltage(_shooterFeederMotor.GetSimVoltage());
 _feederSim.Update(20_ms);
 _shooterFeederMotor.UpdateSimEncoder(_feederSim.GetAngularPosition(), _feederSim.GetAngularVelocity());
}

void SubShooter::UpdatePIDFF() {
  
}

frc2::CommandPtr SubShooter::StartShooter() {
  return Run([this] {
           if (solShooter.Get() == frc::DoubleSolenoid::kReverse) {
             auto FFVolts = _shooterFF.Calculate(ShootFarTarget);
             _shooterMotorMain.SetVoltage(
                 _topPID.Calculate(_topEncoder.GetRate(), ShootFarTarget.value()) * 1_V + FFVolts);
             _secondaryShooterMotor.SetVoltage(
                 _bottomPID.Calculate(_bottomEncoder.GetRate(), ShootFarTarget.value()) * 1_V +
                 FFVolts);
           } else {
             auto FFVolts = _shooterFF.Calculate(ShootCloseTarget);
             _shooterMotorMain.SetVoltage(
                 _topPID.Calculate(_topEncoder.GetRate(), ShootCloseTarget.value()) * 1_V +
                 FFVolts);
             _secondaryShooterMotor.SetVoltage(
                 _bottomPID.Calculate(_bottomEncoder.GetRate(), ShootCloseTarget.value()) * 1_V +
                 FFVolts);
           }
         })
      .AlongWith(WaitUntil([this] { return CheckShooterSpeed(); }));
}

void SubShooter::StopShooterFunc(){
 _shooterMotorMain.Set(0); 
 _secondaryShooterMotor.Set(0);
 _shooterFeederMotor.Set(0);
}

frc2::CommandPtr SubShooter::StopShooterCommand(){
 return RunOnce([this]{ _shooterMotorMain.Set(0);}).AndThen(RunOnce([this] {_secondaryShooterMotor.Set(0);}));
}

frc2::CommandPtr SubShooter::StartFeeder() {
  return RunOnce([this] { _shooterFeederMotor.Set(1); });
}

frc2::CommandPtr SubShooter::ShootSequence() {
  return Sequence(StartShooter(), StartFeeder(), Idle())
      .FinallyDo([this] {StopShooterFunc();});
}

bool SubShooter::CheckShooterSpeed(){
if(units::math::abs(_secondaryShooterMotor.GetVelError()) < 150_rpm && units::math::abs(_shooterMotorMain.GetVelError()) < 150_rpm){
  return true;
 } 
 return false;
}

frc2::CommandPtr SubShooter::ShooterChangePosFar() {
  return RunOnce([this] { solShooter.Set(frc::DoubleSolenoid::kReverse); });
}

frc2::CommandPtr SubShooter::ShooterChangePosClose() {
  return RunOnce([this] { solShooter.Set(frc::DoubleSolenoid::kForward); });
}

frc2::CommandPtr SubShooter::FeedNoteToArm() {
  return Run([this] {
           _shooterFeederMotor.Set(-1);
           _secondaryShooterMotor.Set(-0.2);
           _shooterMotorMain.Set(-0.1);
         })
      .FinallyDo([this] {
        _shooterFeederMotor.Set(0);
        _secondaryShooterMotor.Set(0);
        _shooterMotorMain.Set(0);
      });
}

bool SubShooter::CheckShooterLineBreak() {
  if (_shooterLineBreak.Get() == true){
    return true;
  }

  return false;
}