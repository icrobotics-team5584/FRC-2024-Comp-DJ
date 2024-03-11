// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubClimber.h"
#include "subsystems/SubIntake.h"
#include <frc/RobotBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>

SubClimber::SubClimber() {
    _lClimbMotor.SetConversionFactor(1.0 / gearRatio);
    _lClimbMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    _lClimbMotor.SetPIDFF(lP,lI,lD,lF);
    _lClimbMotor.SetInverted(false);
    _lClimbMotor.SetSoftLimit(rev::CANSparkBase::SoftLimitDirection::kForward, DistanceToTurn(TopHeight).value());
    _lClimbMotor.SetSoftLimit(rev::CANSparkBase::SoftLimitDirection::kReverse, DistanceToTurn(0.02_m).value());

    _rClimbMotor.SetConversionFactor(1.0 / gearRatio);
    _rClimbMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    _rClimbMotor.SetPIDFF(rP,rI,rD,rF);
    _rClimbMotor.SetInverted(true);
    _rClimbMotor.SetSoftLimit(rev::CANSparkBase::SoftLimitDirection::kForward, DistanceToTurn(TopHeight).value());
    _rClimbMotor.SetSoftLimit(rev::CANSparkBase::SoftLimitDirection::kReverse, DistanceToTurn(0.02_m).value()); 

    EnableSoftLimit(true);

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
    frc::SmartDashboard::PutBoolean("Climber/Reseted", Reseted);
    frc::SmartDashboard::PutBoolean("Climber/Reseting", Reseting);
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

// Primary actions
void SubClimber::Start(double power) {
  _lClimbMotor.Set(power);
  _rClimbMotor.Set(power);
}

void SubClimber::Stop() {
  _lClimbMotor.StopMotor();
  _rClimbMotor.StopMotor();
}

void SubClimber::ZeroClimber() {
    _lClimbMotor.SetPosition(0_tr);
    _rClimbMotor.SetPosition(0_tr);
}

double SubClimber::GetLeftCurrent() {
    return _lClimbMotor.GetOutputCurrent();
}

double SubClimber::GetRightCurrent() {
    return _rClimbMotor.GetOutputCurrent();
}

void SubClimber::EnableSoftLimit(bool enabled) {
    _lClimbMotor.EnableSoftLimit(rev::CANSparkBase::SoftLimitDirection::kForward, enabled);
    _lClimbMotor.EnableSoftLimit(rev::CANSparkBase::SoftLimitDirection::kReverse, enabled);
    _rClimbMotor.EnableSoftLimit(rev::CANSparkBase::SoftLimitDirection::kForward, enabled);
    _rClimbMotor.EnableSoftLimit(rev::CANSparkBase::SoftLimitDirection::kReverse, enabled);
}

frc2::CommandPtr SubClimber::ClimberJoystickDrive(frc2::CommandXboxController& _controller) {
    return Run([this, &_controller] {
        _lClimbMotor.Set(-_controller.GetLeftY());
        _rClimbMotor.Set(-_controller.GetRightY());
    }).FinallyDo([this] {
        _lClimbMotor.SetPositionTarget(_lClimbMotor.GetPosition());
        _rClimbMotor.SetPositionTarget(_rClimbMotor.GetPosition());
    });
}

frc2::CommandPtr SubClimber::ClimberJoystickDriveLeft(frc2::CommandXboxController& _controller) {
    return Run([this, &_controller] {
        _lClimbMotor.Set(-_controller.GetLeftY());
    }).FinallyDo([this] {
        _lClimbMotor.SetPositionTarget(_lClimbMotor.GetPosition());
    });
}

frc2::CommandPtr SubClimber::ClimberJoystickDriveRight(frc2::CommandXboxController& _controller) {
    return Run([this, &_controller] {
        _rClimbMotor.Set(-_controller.GetRightY());
    }).FinallyDo([this] {
        _rClimbMotor.SetPositionTarget(_rClimbMotor.GetPosition());
    });
}

//Pointer Commands

frc2::CommandPtr SubClimber::ClimberPosition(units::meter_t distance) {
    return frc2::cmd::RunOnce([this,distance] {SubClimber::GetInstance().DriveToDistance(distance);});
}

frc2::CommandPtr SubClimber::ClimberManualDrive(float power) {
    power = std::clamp(power, -1.0f, 1.0f);
    return frc2::cmd::RunOnce([power] {SubClimber::GetInstance().Start(power);});
}

frc2::CommandPtr SubClimber::ClimberStop() {
    return frc2::cmd::RunOnce([this] {SubClimber::GetInstance().Stop();});
}

frc2::CommandPtr SubClimber::ClimberResetZero() {
    return frc2::cmd::RunOnce([] {SubClimber::GetInstance().ZeroClimber();});
}

frc2::CommandPtr SubClimber::ClimberResetCheck() {
    return frc2::cmd::RunOnce ([this] {ResetLeft = false; ResetRight = false;})
    .AndThen(
    frc2::cmd::Run([this] {
        
        if (GetLeftCurrent() > currentLimit && !ResetLeft) {
            _lClimbMotor.StopMotor(); ResetLeft = true;
        }
        if (GetRightCurrent() > currentLimit && !ResetRight) {
            _rClimbMotor.StopMotor(); ResetRight = true;
        }
        if (ResetLeft && ResetRight) {
            Reseting = false;
        }
    }).Until([this] { return ResetLeft && ResetRight; }));
}

frc2::CommandPtr SubClimber::ClimberAutoReset() {
    return frc2::cmd::RunOnce([this] { Reseting = true; EnableSoftLimit(false);})
        .AndThen(ClimberManualDrive(-0.2))
        .AndThen(frc2::cmd::Wait(0.5_s))
        .AndThen(ClimberResetCheck())
        .AndThen(ClimberResetZero())
        .AndThen(ClimberStop())
        .FinallyDo([this] {Reseting = false; Reseted = true; EnableSoftLimit(true); Stop();});
}

units::meter_t SubClimber::CheckLeftClimberPos() {
  return TurnToDistance(_lClimbMotor.GetPosition());
}

units::meter_t SubClimber::CheckRightClimberPos() {
  return TurnToDistance(_rClimbMotor.GetPosition());
}