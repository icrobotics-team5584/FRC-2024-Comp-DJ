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
};

void SubClimber::Periodic() {

}

void SubClimber::SimulationPeriodic() {
    frc::SmartDashboard::PutData("Climber/Mech Display", &mech);
    frc::SmartDashboard::PutNumber("Climber/Left distance", TurnToDistance(lClimbMotor.GetPosition()).value());
    frc::SmartDashboard::PutNumber("Climber/Right distance", TurnToDistance(rClimbMotor.GetPosition()).value());
    frc::SmartDashboard::PutNumber("Climber/Target distance", TargetDistance.value());

    frc::SmartDashboard::PutNumber("Climber/Elv sim position", lElvSim.GetPosition().value());
    frc::SmartDashboard::PutNumber("Climber/Elv sim velocity", lElvSim.GetVelocity().value());

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

void SubClimber::SetTarget(units::meter_t Distance) {
    TargetDistance = Distance;
}

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
    SetTarget(distance);
    lClimbMotor.SetPositionTarget(DistanceToTurn(distance));
    rClimbMotor.SetPositionTarget(DistanceToTurn(distance));
}

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

using namespace frc2::cmd;

frc2::CommandPtr SubClimber::ClimberExtend() {
    return RunOnce([] {SubClimber::GetInstance().Extend();});
}

frc2::CommandPtr SubClimber::ClimberRetract() {
    return RunOnce([] {SubClimber::GetInstance().Retract();});
}

frc2::CommandPtr SubClimber::ClimberExtendManual() {
    return RunOnce([] {SubClimber::GetInstance().Start(0.5);});
}

frc2::CommandPtr SubClimber::ClimberRetractManual() {
    return RunOnce([] {SubClimber::GetInstance().Start(-0.5);});
}

frc2::CommandPtr SubClimber::ClimberStop() {
    return RunOnce([] {SubClimber::GetInstance().Stop();});
}