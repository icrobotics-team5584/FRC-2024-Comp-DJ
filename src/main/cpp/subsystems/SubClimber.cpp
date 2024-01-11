// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubClimber.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkMaxAbsoluteEncoder.h>

SubClimber::SubClimber() {
    lClimbMotor.SetConversionFactor(1);
    lClimbMotor.SetPIDFF(lP,lI,lD,lF);
    lClimbMotor.ConfigSmartMotion(MaxVelocity,MaxAcceleration,Tolerance);
    // lClimbMotor.UseAbsoluteEncoder(lClimbEncoder);
    rClimbMotor.SetConversionFactor(1);
    rClimbMotor.SetPIDFF(rP,rI,rD,rF);
    rClimbMotor.ConfigSmartMotion(MaxVelocity,MaxAcceleration,Tolerance);
    // rClimbMotor.UseAbsoluteEncoder(rClimbEncoder);

    lClimbMotor.SetInverted(true);
    // lClimbEncoder.SetInverted(true);
    frc::SmartDashboard::PutNumber("Climber/test", 0);

    // lClimbEncoder.SetZeroOffset(0);
    // rClimbEncoder.SetZeroOffset(0);

    lClimbMotor.SetPosition(20_deg);
};

// This method will be called once per scheduler run
void SubClimber::Periodic() {
    frc::SmartDashboard::PutData("Climber/Mech Display", &mech);
}

void SubClimber::SimulationPeriodic() {
    frc::SmartDashboard::PutNumber("Climber/Left position", lClimbMotor.GetPosition().value());
    frc::SmartDashboard::PutNumber("Climber/Left position target", lClimbMotor.GetPositionTarget().value());
    frc::SmartDashboard::PutNumber("Climber/Right position", rClimbMotor.GetPosition().value());
    frc::SmartDashboard::PutNumber("Climber/Right position target", rClimbMotor.GetPositionTarget().value());
    frc::SmartDashboard::PutNumber("Climber/Left velocity", lClimbMotor.GetVelocity().value());
    frc::SmartDashboard::PutNumber("Climber/Right velocity", rClimbMotor.GetVelocity().value());
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

    mechLeftElevator->SetLength(lClimbMotor.GetPosition().value() / 20);
    mechRightElevator->SetLength(rClimbMotor.GetPosition().value() / -20);
    mechTar->SetLength(TargetDistance.value());
}

void SubClimber::SetTarget(units::meter_t Distance) {
    TargetDistance = Distance;
}

void SubClimber::Retract() {
    SetTarget(1_m);
    lClimbMotor.SetPositionTarget(20_deg);
    rClimbMotor.SetSmartMotionTarget(20_deg);
    frc::SmartDashboard::PutNumber("Climber/test", 1);
}

void SubClimber::Extend() {
    SetTarget(4_m);
    lClimbMotor.SetPositionTarget(80_deg);
    rClimbMotor.SetSmartMotionTarget(80_deg);
    frc::SmartDashboard::PutNumber("Climber/test", 2);
}