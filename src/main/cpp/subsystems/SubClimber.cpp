// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubClimber.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkMaxAbsoluteEncoder.h>

SubClimber::SubClimber() {
    lClimbMotor.SetConversionFactor(1 / GearRatio);
    lClimbMotor.SetPIDFF(lP,lI,lD,lF);
    lClimbMotor.SetInverted(true);
    lClimbMotor.ConfigSmartMotion(MaxVelocity,MaxAcceleration,Tolerance);
    rClimbMotor.SetConversionFactor(1 / GearRatio);
    rClimbMotor.SetPIDFF(rP,rI,rD,rF);
    rClimbMotor.ConfigSmartMotion(MaxVelocity,MaxAcceleration,Tolerance);
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

    auto volts = lClimbMotor.GetSimVoltage();
    lSim.SetInputVoltage(volts);
    lSim.Update(20_ms);
    auto angle = lSim.GetAngularPosition();
    auto velocity = lSim.GetAngularVelocity();
    lClimbMotor.UpdateSimEncoder(angle,velocity);

    volts = rClimbMotor.GetSimVoltage();
    rSim.SetInputVoltage(volts);
    rSim.Update(20_ms);
    angle = rSim.GetAngularPosition();
    velocity = rSim.GetAngularVelocity();
    rClimbMotor.UpdateSimEncoder(angle,velocity);

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

units::meter_t SubClimber::TurnToDistance(units::turn_t turn) {
    return turn.value() * WheelCir;
};

void SubClimber::DriveToDistance(units::meter_t distance) {
    SetTarget(distance);
    lClimbMotor.SetSmartMotionTarget(DistanceToTurn(distance-BaseHeight));
    rClimbMotor.SetSmartMotionTarget(DistanceToTurn(distance-BaseHeight));
}

void SubClimber::Retract() {
    DriveToDistance(0_m);
}

void SubClimber::Extend() {
    DriveToDistance(1.3_m);
}