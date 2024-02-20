// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubClimber.h"
#include <frc/RobotBase.h>
#include <frc/smartdashboard/SmartDashboard.h>

SubClimber::SubClimber() {
    _lClimbMotor.SetConversionFactor(1.0 / gearRatio);
    _lClimbMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    _lClimbMotor.SetPIDFF(lP,lI,lD,lF);
    _lClimbMotor.SetInverted(false);
    // _lClimbMotor.SetSoftLimit(rev::CANSparkBase::SoftLimitDirection::kForward, DistanceToTurn(TopHeight).value());

    _rClimbMotor.SetConversionFactor(1.0 / gearRatio);
    _rClimbMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    _rClimbMotor.SetPIDFF(rP,rI,rD,rF);
    _rClimbMotor.SetInverted(true);
    // _rClimbMotor.SetSoftLimit(rev::CANSparkBase::SoftLimitDirection::kForward, DistanceToTurn(TopHeight).value());

    LockCylinder.Set(frc::DoubleSolenoid::Value::kReverse);

    frc::SmartDashboard::PutData("Climber/Left motor", (wpi::Sendable*)&_lClimbMotor);
    frc::SmartDashboard::PutData("Climber/Right motor", (wpi::Sendable*)&_rClimbMotor);
    frc::SmartDashboard::PutData("Climber/Lock Cylinder", (wpi::Sendable*)&LockCylinder);
};

void SubClimber::Periodic() {
    frc::SmartDashboard::PutNumber("Climber/Left distance", TurnToDistance(_lClimbMotor.GetPosition()).value());
    frc::SmartDashboard::PutNumber("Climber/Right distance", TurnToDistance(_rClimbMotor.GetPosition()).value());
    frc::SmartDashboard::PutNumber("Climber/Target distance", TargetDistance.value());
    frc::SmartDashboard::PutNumber("Climber/Left current", _lClimbMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Climber/Right current", _rClimbMotor.GetOutputCurrent());
    frc::SmartDashboard::PutBoolean("Climber/Reset?", Reseted);

    // if (TurnToDistance(_lClimbMotor.GetPosition()) > TopHeight || TurnToDistance(_rClimbMotor.GetPosition()) > TopHeight) {
    //     if (_lClimbMotor.GetVelocity().value() > 0 || _rClimbMotor.GetVelocity().value() > 0) {
    //         Stop();
    //     }
    // }
    // if (!Reseting) {
    //     if (TurnToDistance(_lClimbMotor.GetPosition()) < 0.1_m || TurnToDistance(_rClimbMotor.GetPosition()) < 0.1_m) {
    //         if (_lClimbMotor.GetVelocity().value() < 0 || _rClimbMotor.GetVelocity().value() < 0) {
    //             Stop();
    //         }
    //     }
    // }
}

void SubClimber::SimulationPeriodic() {
    frc::SmartDashboard::PutData("Climber/Mech Display", &mech);
    frc::SmartDashboard::PutNumber("Climber/Left sim distance", TurnToDistance(_lClimbMotor.GetPosition()).value());
    frc::SmartDashboard::PutNumber("Climber/Right sim distance", TurnToDistance(_rClimbMotor.GetPosition()).value());

  lElvSim.SetInputVoltage(_lClimbMotor.GetSimVoltage());
  lElvSim.Update(20_ms);
  _lClimbMotor.UpdateSimEncoder(DistanceToTurn(lElvSim.GetPosition()),
                               DistanceToTurn(lElvSim.GetVelocity()));

    rElvSim.SetInputVoltage(_rClimbMotor.GetSimVoltage());
    rElvSim.Update(20_ms);
    _rClimbMotor.UpdateSimEncoder(DistanceToTurn(rElvSim.GetPosition()), DistanceToTurn(rElvSim.GetVelocity()));

    mechLeftElevator->SetLength(TurnToDistance(_lClimbMotor.GetPosition()).value());
    mechRightElevator->SetLength(TurnToDistance(_rClimbMotor.GetPosition()).value());
    mechTar->SetLength(TargetDistance.value());
}

// Units translation

units::turn_t SubClimber::DistanceToTurn(units::meter_t distance) {
  return distance / WheelCir * 1_tr;
}

units::radians_per_second_t SubClimber::DistanceToTurn(units::meters_per_second_t distance) {
    return distance / WheelCir * 1_tr;
}

units::meter_t SubClimber::TurnToDistance(units::turn_t turn) {
  return turn.value() * WheelCir;
};

void SubClimber::DriveToDistance(units::meter_t distance) {
    if (LockCylinder.Get() != frc::DoubleSolenoid::Value::kForward) {
        TargetDistance = distance;
        _lClimbMotor.SetPositionTarget(DistanceToTurn(distance));
        _rClimbMotor.SetPositionTarget(DistanceToTurn(distance));
    }
}

//Secondary move commands

void SubClimber::Retract() {
    DriveToDistance(BaseHeight);
}

void SubClimber::Extend() {
    DriveToDistance(1.3_m);
}

void SubClimber::Start(double power) {
  _lClimbMotor.Set(power);
  _rClimbMotor.Set(power);
}

void SubClimber::Stop() {
  _lClimbMotor.Set(0);
  _rClimbMotor.Set(0);
}

void SubClimber::Lock() {
    Stop();
    LockCylinder.Set(frc::DoubleSolenoid::Value::kForward);
}

void SubClimber::Unlock() {
    LockCylinder.Set(frc::DoubleSolenoid::Value::kReverse);
}

void SubClimber::ZeroClimber() {
    _lClimbMotor.SetPosition(0_tr);
    _rClimbMotor.SetPosition(0_tr);
}

double SubClimber::GetCurrent() {
    // return std::max(_lClimbMotor.GetOutputCurrent(),_rClimbMotor.GetOutputCurrent());
    return _lClimbMotor.GetOutputCurrent();
}

//Pointer Commands

frc2::CommandPtr SubClimber::ClimberExtend() {
    return frc2::cmd::RunOnce([] {SubClimber::GetInstance().Extend();});
}

frc2::CommandPtr SubClimber::ClimberRetract() {
    return frc2::cmd::RunOnce([] {SubClimber::GetInstance().Retract();});
}

frc2::CommandPtr SubClimber::ClimberPosition(units::meter_t distance) {
    return frc2::cmd::RunOnce([distance] {SubClimber::GetInstance().DriveToDistance(distance);});
}

frc2::CommandPtr SubClimber::ClimberManualDrive(float power) {
    power = std::clamp(power, -1.0f, 1.0f);
    return frc2::cmd::RunOnce([power] {SubClimber::GetInstance().Start(power);});
}

frc2::CommandPtr SubClimber::ClimberStop() {
    return frc2::cmd::RunOnce([] {SubClimber::GetInstance().Stop();});
}

frc2::CommandPtr SubClimber::ClimberLock() {
    return frc2::cmd::RunOnce([] {SubClimber::GetInstance().Lock();});
}

frc2::CommandPtr SubClimber::ClimberUnlock() {
    return frc2::cmd::RunOnce([] {SubClimber::GetInstance().Unlock();});
}

frc2::CommandPtr SubClimber::ClimberResetZero() {
    return frc2::cmd::RunOnce([] {SubClimber::GetInstance().ZeroClimber();});
}

frc2::CommandPtr SubClimber::ClimberAutoReset() {
    return RunOnce([this] {Reseting = true;}).AndThen(ClimberManualDrive(-0.2)).AndThen(frc2::cmd::Wait(0.5_s))
    .AndThen(frc2::cmd::WaitUntil([this] { return GetCurrent() > currentLimit;}))
    .AndThen(ClimberStop()).AndThen(ClimberResetZero()).AndThen(ClimberPosition(0.2_m))
    .AndThen(RunOnce([this] {Reseting = false; Reseted = true;}));
}