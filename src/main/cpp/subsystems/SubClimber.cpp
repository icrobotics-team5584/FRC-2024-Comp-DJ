// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubClimber.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkMaxAbsoluteEncoder.h>

SubClimber::SubClimber() {
    lClimbMotor.SetConversionFactor(1.0 / gearRatio);
    lClimbMotor.SetPIDFF(lP,lI,lD,lF);
    lClimbMotor.SetInverted(true);
    rClimbMotor.SetConversionFactor(1.0 / gearRatio);
    rClimbMotor.SetPIDFF(rP,rI,rD,rF);
    rClimbMotor.SetInverted(false);

    frc::SmartDashboard::PutData("Climber/Left motor", (wpi::Sendable*)&lClimbMotor);
    frc::SmartDashboard::PutData("Climber/Right motor", (wpi::Sendable*)&rClimbMotor);
    frc::SmartDashboard::PutData("Climber/Lock Cylinder", (wpi::Sendable*)&LockCylinder);
};

void SubClimber::Periodic() {
    frc::SmartDashboard::PutNumber("Climber/Left distance", TurnToDistance(lClimbMotor.GetPosition()).value());
    frc::SmartDashboard::PutNumber("Climber/Right distance", TurnToDistance(rClimbMotor.GetPosition()).value());
    frc::SmartDashboard::PutNumber("Climber/Target distance", TargetDistance.value());
}

void SubClimber::SimulationPeriodic() {
    frc::SmartDashboard::PutData("Climber/Mech Display", &mech);

    lElvSim.SetInputVoltage(lClimbMotor.GetSimVoltage());
    lElvSim.Update(20_ms);
    lClimbMotor.UpdateSimEncoder(DistanceToTurn(lElvSim.GetPosition()), DistanceToTurn(lElvSim.GetVelocity()));

    rElvSim.SetInputVoltage(rClimbMotor.GetSimVoltage());
    rElvSim.Update(20_ms);
    rClimbMotor.UpdateSimEncoder(DistanceToTurn(rElvSim.GetPosition()), DistanceToTurn(rElvSim.GetVelocity()));

    mechLeftElevator->SetLength(TurnToDistance(lClimbMotor.GetPosition()).value());
    mechRightElevator->SetLength(TurnToDistance(rClimbMotor.GetPosition()).value());
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
    if (LockCylinder.Get() = 1) {
        TargetDistance = distance;
        lClimbMotor.SetPositionTarget(DistanceToTurn(distance));
        rClimbMotor.SetPositionTarget(DistanceToTurn(distance));
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
    lClimbMotor.Set(power);
    rClimbMotor.Set(power);
}

void SubClimber::Stop() {
    lClimbMotor.Set(0);
    rClimbMotor.Set(0);
}

void SubClimber::Lock() {
    Stop();
    LockCylinder.Set(frc::DoubleSolenoid::Value::kForward);
}

void SubClimber::Unlock() {
    LockCylinder.Set(frc::DoubleSolenoid::Value::kReverse);
}

//Pointer Commands

frc2::CommandPtr SubClimber::ClimberExtend() {
    return frc2::cmd::RunOnce([] {SubClimber::GetInstance().Extend();});
}

frc2::CommandPtr SubClimber::ClimberRetract() {
    return frc2::cmd::RunOnce([] {SubClimber::GetInstance().Retract();});
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