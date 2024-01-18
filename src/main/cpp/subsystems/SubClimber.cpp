// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubClimber.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkMaxAbsoluteEncoder.h>

SubClimber::SubClimber() {
    lClimbMotor.SetConversionFactor(1 / GearRatio);
    lClimbMotor.SetPIDFF(lP,lI,lD,lF);
    lClimbMotor.SetInverted(false);
    lClimbMotor.ConfigSmartMotion(MaxVelocity,MaxAcceleration,Tolerance);
    // lClimbMotor.UseAbsoluteEncoder(lClimbEncoder);
    rClimbMotor.SetConversionFactor(1 / GearRatio);
    rClimbMotor.SetPIDFF(rP,rI,rD,rF);
    rClimbMotor.ConfigSmartMotion(MaxVelocity,MaxAcceleration,Tolerance);
    rClimbMotor.SetInverted(false);
    // rClimbMotor.UseAbsoluteEncoder(rClimbEncoder);

    // lClimbEncoder.SetInverted(true);
    frc::SmartDashboard::PutNumber("Climber/test", 0);

    frc::SmartDashboard::PutData("Climber/Left motor", (wpi::Sendable*)&lClimbMotor);
    frc::SmartDashboard::PutData("Climber/Right motor", (wpi::Sendable*)&rClimbMotor);
};

// This method will be called once per scheduler run
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

void SubClimber::Retract() {
    // if (currentPosition == 4) {
        SetTarget(0_m);
        lClimbMotor.SetSmartMotionTarget(0_deg);
        rClimbMotor.SetSmartMotionTarget(0_deg);
        frc::SmartDashboard::PutNumber("Climber/test", 1);
        // currentPosition = 0;
    // }
}

void SubClimber::Extend() {
    SetTarget(1.3_m);
    lClimbMotor.SetSmartMotionTarget(DistanceToTurn(1.3_m));
    rClimbMotor.SetSmartMotionTarget(DistanceToTurn(1.3_m));
    frc::SmartDashboard::PutNumber("Climber/test", 2);
    // currentPosition = 4;
}

void SubClimber::UpliftMiddle() {
    // if (currentPosition == 4) {
        SetTarget(0.6_m);
        lClimbMotor.SetPositionTarget(DistanceToTurn(0.6_m));
        rClimbMotor.SetPositionTarget(DistanceToTurn(0.6_m));
        currentPosition = 2;
    // }
}

units::turn_t SubClimber::DistanceToTurn(units::meter_t distance) {
    return distance / WheelCir * 1_tr;
}

units::meter_t SubClimber::TurnToDistance(units::turn_t turn) {
    return turn.value() * WheelCir;
};