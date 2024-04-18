// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubShooter.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "utilities/BotVars.h"

SubShooter::SubShooter() {

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

  // Calculate and filter velocity measurements
  // (we were having issues with the WPILib encoder's GetRate() readings)
  _bottomEncoderDiff = (_bottomEncoder.GetDistance()-_bottomEncoderPositionPrev)/0.02;
  _topEncoderDiff = (_topEncoder.GetDistance()-_topEncoderPositionPrev)/0.02;

  _bottomEncoderPositionPrev = _bottomEncoder.GetDistance();
  _topEncoderPositionPrev = _topEncoder.GetDistance();


  _topPastVelocityMeasurements[2] = _topPastVelocityMeasurements[1];
  _topPastVelocityMeasurements[1] = _topPastVelocityMeasurements[0];
  _topPastVelocityMeasurements[0] = _topEncoderDiff;

  _bottomPastVelocityMeasurements[2] = _bottomPastVelocityMeasurements[1];
  _bottomPastVelocityMeasurements[1] = _bottomPastVelocityMeasurements[0];
  _bottomPastVelocityMeasurements[0] = _bottomEncoderDiff;


  _topPastVelocityAvg = (_topPastVelocityMeasurements[0] + _topPastVelocityMeasurements[1] + _topPastVelocityMeasurements[2])/3.0;
  _bottomPastVelocityAvg = (_bottomPastVelocityMeasurements[0] + _bottomPastVelocityMeasurements[1] + _bottomPastVelocityMeasurements[2])/3.0;

  UpdatePIDFF();


  // Log subsystem info
  frc::SmartDashboard::PutNumber("Shooter/Piston Position", solShooter.Get());

  if (solShooter.Get() == frc::DoubleSolenoid::kReverse) {
    frc::SmartDashboard::PutString("Shooter/Angle: ", "Score From Podium");
  } else {
    frc::SmartDashboard::PutString("Shooter/Angle: ", "Score From Subwoofer");
  }

  frc::SmartDashboard::PutBoolean("Shooter/Shooter linebreaker", _shooterLineBreak.Get());
  frc::SmartDashboard::PutNumber("Shooter/Top Encoder", _topEncoder.GetRate());
  frc::SmartDashboard::PutNumber("Shooter/Top Shooter Distance", _topEncoder.GetDistance());
  frc::SmartDashboard::PutNumber("Shooter/Top Shooter Distance Prev", _topEncoderDiff);
  frc::SmartDashboard::PutNumber("Shooter/Bottom Encoder", _bottomEncoder.GetRate());
  frc::SmartDashboard::PutNumber("Shooter/Bottom Shooter Distance", _bottomEncoder.GetDistance());
  frc::SmartDashboard::PutNumber("Shooter/Bottom Shooter Distance Prev", (_bottomEncoderDiff));
  frc::SmartDashboard::PutNumber("Shooter/Bottom Velocity Error", std::abs(ShootFarTarget.value() - _bottomEncoderDiff));
  frc::SmartDashboard::PutNumber("Shooter/Top Velocity Error", std::abs(ShootFarTarget.value() - _topEncoderDiff));
  frc::SmartDashboard::PutNumber("Shooter/Top velocity avg", _topPastVelocityAvg);
  frc::SmartDashboard::PutNumber("Shooter/Bottom velocity avg", _bottomPastVelocityAvg);
  frc::SmartDashboard::PutBoolean("Shooter/CheckShoote boolean", CheckShooterSpeed());
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
  auto FFVolts = _shooterFF.Calculate(CurrentShooterTarget);
  auto Topvolts = _topPID.Calculate(_topPastVelocityAvg, CurrentShooterTarget.value()) * 1_V + FFVolts;
  if (Topvolts < 0_V){
    Topvolts = 0_V;
  }
  _topShooterMotor.SetVoltage(Topvolts);

  auto Bottomvolts = _bottomPID.Calculate(_bottomPastVelocityAvg, CurrentShooterTarget.value()) * 1_V + FFVolts;
  if (Bottomvolts < 0_V){
    Bottomvolts = 0_V;
  }
  _bottomShooterMotor.SetVoltage(Bottomvolts);
}

//Set shooter pid targets to speaker speeds until the shooter is at speed, then cancel. Does not change shooter pid targets on cancelled
frc2::CommandPtr SubShooter::StartShooter() {
  return Run([this] {
           if (solShooter.Get() == frc::DoubleSolenoid::kReverse) {
             CurrentShooterTarget = ShootFarTarget;
           } else {
             CurrentShooterTarget = ShootCloseTarget;
           }
         })
      .Until([this] { return CheckShooterSpeed(); });
}

//Move shooter to up position, set shooter pid targets to amp speeds until shooter is at speed, then cancel command. Doesn't change pid targets back to 0
frc2::CommandPtr SubShooter::ShootIntoAmp() {
  return ShooterChangePosClose()
      .AndThen(RunOnce([this] { CurrentShooterTarget = ShootAmpTarget; }))
      .AndThen(WaitUntil([this] { return CheckShooterSpeed(); }));
}
  
//Sets shooter pid target, then starts the shooter feeders and waits until the command is cancelled, then stop spinning shooter
frc2::CommandPtr SubShooter::ShootIntoAmpSequence() {
    return Sequence(ShootIntoAmp(), StartFeeder(), Idle())
      .FinallyDo([this] {StopShooterFunc();});
}

//Set shooter pid target to 0 and turn off shooter feeder motor
void SubShooter::StopShooterFunc(){
 CurrentShooterTarget = 0_tps;
 _shooterFeederMotor.Set(0);
}

//Set shooter speeds to 0% power
frc2::CommandPtr SubShooter::StopShooterCommand(){
 return RunOnce([this]{ _topShooterMotor.Set(0);}).AndThen(RunOnce([this] {_bottomShooterMotor.Set(0);}));
}

//Set shooter feeder motor to 100% power
frc2::CommandPtr SubShooter::StartFeeder() {
  return RunOnce([this] { _shooterFeederMotor.Set(1); });
}

//Set shooter feeder motor to 50% power
frc2::CommandPtr SubShooter::StartFeederSlow(){
  return Run([this]{ _shooterFeederMotor.Set(0.5);});
}

//Reverse the shooter feeder motor and then set it to 0% power when command is cancelled
frc2::CommandPtr SubShooter::ReverseFeeder() {
  return Run([this] { _shooterFeederMotor.Set(-0.2); }).FinallyDo([this] {
    _shooterFeederMotor.Set(0);
  });
}

//Set shooter feeder motor to 0% power
frc2::CommandPtr SubShooter::StopFeeder() {
  return RunOnce([this] {_shooterFeederMotor.Set(0);});
}

//Function to set shooter feeder motor to 0% power
void SubShooter::StopFeederFunc() {
  _shooterFeederMotor.Set(0);
}

//Start the shooter, then once shooter is at speed, feed note to shooter and wait until command is cancelled, then stop the shooter
frc2::CommandPtr SubShooter::ShootSequence() {
  return Sequence(StartShooter(), StartFeeder(), Idle())
      .FinallyDo([this] {StopShooterFunc();});
}

//The shooter sequenced used in auton paths
frc2::CommandPtr SubShooter::AutoShootSequence() {
  return Sequence(StartShooter().WithTimeout(0.25_s), StartFeeder());
}

//Check if the shooter speeds are within tolerance
bool SubShooter::CheckShooterSpeed(){
  if (std::abs(CurrentShooterTarget.value() - _bottomPastVelocityAvg) < 3 &&
      std::abs(CurrentShooterTarget.value() - _topPastVelocityAvg) < 3) {
    return true;
  }
 return false;
}

//Change shooter to down position
frc2::CommandPtr SubShooter::ShooterChangePosFar() {
  return RunOnce([this] { solShooter.Set(frc::DoubleSolenoid::kReverse); });
}

//Change shooter to up position
frc2::CommandPtr SubShooter::ShooterChangePosClose() {
  return RunOnce([this] { solShooter.Set(frc::DoubleSolenoid::kForward); });
}

//Deprecated
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

//Spin shooter wheels towards intake, used during outtake as a failsafe
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

frc2::CommandPtr SubShooter::ShooterPassNote(){
  return RunOnce([this] { CurrentShooterTarget = ShootPassNoteTarget; })
      .AndThen(WaitUntil([this] { return CheckShooterSpeed(); }));
}

frc2::CommandPtr SubShooter::ShooterPassNoteSequence() {
      return Sequence(ShooterPassNote(), StartFeeder(), Idle())
      .FinallyDo([this] {StopShooterFunc();});
}

//Check if there is a note in the shooter
bool SubShooter::CheckShooterLineBreak() {
  if(_shooterLineBreak.Get() == BotVars::Choose(false, true)){
    return true;
  }

  return false;
}

//Deprecated
frc2::CommandPtr SubShooter::IntakeFromSource() {
  return Run([this] {
           _topShooterMotor.Set(-0.3);
           _bottomShooterMotor.Set(-0.3);
           _shooterFeederMotor.Set(-1);
         })
      .FinallyDo([this] {
        _topShooterMotor.Set(0);
        _bottomShooterMotor.Set(0);
        _shooterFeederMotor.Set(0);
      });
}