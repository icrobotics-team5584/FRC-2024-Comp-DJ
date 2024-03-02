// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubShooter.h"
#include <frc/smartdashboard/SmartDashboard.h>

SubShooter::SubShooter() {
  solShooter.Set(frc::DoubleSolenoid::kReverse);

  _bottomShooterMotor.SetInverted(true);
  _topShooterMotor.SetInverted(true);
  _topEncoder.SetReverseDirection(true);
  _bottomEncoder.SetReverseDirection(true);
  frc::SmartDashboard::PutData("Shooter/Top Motor", (wpi::Sendable*)&_topShooterMotor);
  frc::SmartDashboard::PutData("Shooter/Bottom Motor", (wpi::Sendable*)&_bottomShooterMotor);
  frc::SmartDashboard::PutData("Shooter/Feeder motor", (wpi::Sendable*)&_shooterFeederMotor);
  frc::SmartDashboard::PutData("Shooter/Top PID", (wpi::Sendable*)&_topPID);
  frc::SmartDashboard::PutData("Shooter/Bottom PID", (wpi::Sendable*)&_bottomPID);

  _topEncoder.SetSamplesToAverage(30);
  _topEncoder.SetDistancePerPulse(1.00/2048.00);
  _bottomEncoder.SetSamplesToAverage(30);
  _bottomEncoder.SetDistancePerPulse(1.00/2048.00);
 
  _shooterFeederMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  frc::SmartDashboard::PutData("Shooter/Top Motor", (wpi::Sendable*)&_topShooterMotor);
  frc::SmartDashboard::PutData("Shooter/Bottom Motor", (wpi::Sendable*)&_bottomShooterMotor);
  frc::SmartDashboard::PutData("Shooter/Feeder motor", (wpi::Sendable*)&_shooterFeederMotor);
  

  _shooterFeederMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus0, 500);
  _shooterFeederMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus1, 20);
  _shooterFeederMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus2, 500);
  _shooterFeederMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus3, 500);
  _shooterFeederMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus4, 500);
  _shooterFeederMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus5, 500);
  _shooterFeederMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus6, 500);
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

  frc::SmartDashboard::PutBoolean("Shooter/Shooter linebreaker", _shooterLineBreak.Get());
  frc::SmartDashboard::PutNumber("Shooter/Top Encoder", _topEncoder.GetRate());
  frc::SmartDashboard::PutNumber("Shooter/Top Shooter Distance", _topEncoder.GetDistance());
  frc::SmartDashboard::PutNumber("Shooter/Top Shooter Distance Prev", (_topEncoder.GetDistance()-_topEncoderPositionPrev)/0.02);
  frc::SmartDashboard::PutNumber("Shooter/Bottom Encoder", _bottomEncoder.GetRate());
  frc::SmartDashboard::PutNumber("Shooter/Bottom Shooter Distance", _bottomEncoder.GetDistance());
  frc::SmartDashboard::PutNumber("Shooter/Bottom Shooter Distance Prev", (_bottomEncoder.GetDistance()-_bottomEncoderPositionPrev)/0.02);

  _bottomEncoderPositionPrev = _bottomEncoder.GetDistance();
  _topEncoderPositionPrev = _topEncoder.GetDistance();
}

void SubShooter::SimulationPeriodic(){
 _topShooterSim.SetInputVoltage(_topShooterMotor.GetSimVoltage());
 _topShooterSim.Update(20_ms);
 _topShooterMotor.UpdateSimEncoder(_topShooterSim.GetAngularPosition(), _topShooterSim.GetAngularVelocity());
 _topEncoderSim.SetDistance(_topShooterSim.GetAngularPosition().convert<units::turns>().value());
 _topEncoderSim.SetRate(_topShooterSim.GetAngularVelocity().convert<units::turns_per_second>().value());

 _bottomShooterSim.SetInputVoltage(_bottomShooterMotor.GetSimVoltage());
 _bottomShooterSim.Update(20_ms);
 _bottomShooterMotor.UpdateSimEncoder(_bottomShooterSim.GetAngularPosition(), _bottomShooterSim.GetAngularVelocity());
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
             _topShooterMotor.SetVoltage(
                 _topPID.Calculate((_topEncoder.GetDistance()-_topEncoderPositionPrev)/0.02, ShootFarTarget.value()) * 1_V + FFVolts);
             _bottomShooterMotor.SetVoltage(
                 _bottomPID.Calculate((_bottomEncoder.GetDistance()-_bottomEncoderPositionPrev)/0.02, ShootFarTarget.value()) * 1_V +
                 FFVolts);
           } else {
             auto FFVolts = _shooterFF.Calculate(ShootCloseTarget);
             _topShooterMotor.SetVoltage(
                 _topPID.Calculate((_topEncoder.GetDistance()-_topEncoderPositionPrev)/0.02, ShootCloseTarget.value()) * 1_V +
                 FFVolts);
             _bottomShooterMotor.SetVoltage(
                 _bottomPID.Calculate((_bottomEncoder.GetDistance()-_bottomEncoderPositionPrev)/0.02, ShootCloseTarget.value()) * 1_V +
                 FFVolts);
           }
         })
      .AlongWith(WaitUntil([this] { return CheckShooterSpeed(); }));
}

void SubShooter::StopShooterFunc(){
 _topShooterMotor.Set(0); 
 _bottomShooterMotor.Set(0);
 _shooterFeederMotor.Set(0);
}

frc2::CommandPtr SubShooter::StopShooterCommand(){
 return RunOnce([this]{ _topShooterMotor.Set(0);}).AndThen(RunOnce([this] {_bottomShooterMotor.Set(0);}));
}

frc2::CommandPtr SubShooter::StartFeeder() {
  return RunOnce([this] { _shooterFeederMotor.Set(1); });
}

frc2::CommandPtr SubShooter::StartFeederSlow(){
  return Run([this]{ _shooterFeederMotor.Set(1);});
}

frc2::CommandPtr SubShooter::ReverseFeeder() {
  return Run([this] { _shooterFeederMotor.Set(-0.2); }).WithTimeout(0.2_s).FinallyDo([this] {
    _shooterFeederMotor.Set(0);
  });
}

frc2::CommandPtr SubShooter::StopFeeder() {
  return RunOnce([this] {_shooterFeederMotor.Set(0);});
}

void SubShooter::StopFeederFunc() {
  _shooterFeederMotor.Set(0);
}

frc2::CommandPtr SubShooter::ShootSequence() {
  return Sequence(StartShooter(), StartFeeder(), Idle())
      .FinallyDo([this] {StopShooterFunc();});
}

frc2::CommandPtr SubShooter::AutoShootSequence() {
  return Sequence(StartShooter().WithTimeout(0.25_s), StartFeeder());
}


bool SubShooter::CheckShooterSpeed(){
if(units::math::abs(_bottomShooterMotor.GetVelError()) < 500_rpm && units::math::abs(_topShooterMotor.GetVelError()) < 500_rpm){
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
           _bottomShooterMotor.Set(-0.2);
           _topShooterMotor.Set(-0.1);
         })
      .FinallyDo([this] {
        _shooterFeederMotor.Set(0);
        _bottomShooterMotor.Set(0);
        _topShooterMotor.Set(0);});
}
frc2::CommandPtr SubShooter::Outtake() {
  return Run([this] {
           _shooterFeederMotor.Set(-1);
           _bottomShooterMotor.Set(-0.1);
           _topShooterMotor.Set(-0.1);
         })
      .FinallyDo([this] {
        _bottomShooterMotor.Set(0);
        _topShooterMotor.Set(0);
        _shooterFeederMotor.Set(0);
      });
}

bool SubShooter::CheckShooterLineBreak() {
  if(_shooterLineBreak.Get() == true){
    return true;
  }

  return false;
}