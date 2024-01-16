// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubClimber.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkMaxAbsoluteEncoder.h>

SubClimber::SubClimber() {
    lClimbMotor.SetConversionFactor(1 / GearRatio);
    lClimbMotor.SetPIDFF(lP,lI,lD,lF);
    lClimbMotor.ConfigSmartMotion(MaxVelocity,MaxAcceleration,Tolerance);
    // lClimbMotor.UseAbsoluteEncoder(lClimbEncoder);
    rClimbMotor.SetConversionFactor(1 / GearRatio);
    rClimbMotor.SetPIDFF(rP,rI,rD,rF);
    rClimbMotor.ConfigSmartMotion(MaxVelocity,MaxAcceleration,Tolerance);
    // rClimbMotor.UseAbsoluteEncoder(rClimbEncoder);

    lClimbMotor.SetInverted(true);
    // lClimbEncoder.SetInverted(true);
    frc::SmartDashboard::PutNumber("Climber/test", 0);
    

};

// This method will be called once per scheduler run
void SubClimber::Periodic() {

}

void SubClimber::SimulationPeriodic() {
    frc::SmartDashboard::PutData("Climber/Mech Display", &mech);
    frc::SmartDashboard::PutNumber("Climber/Left position", lClimbMotor.GetPosition().value());
    frc::SmartDashboard::PutNumber("Climber/Left position target", lClimbMotor.GetPositionTarget().value());
    frc::SmartDashboard::PutNumber("Climber/Left distance", TurnToDistance(lClimbMotor.GetPosition()).value());
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

    mechLeftElevator->SetLength(TurnToDistance(lClimbMotor.GetPosition()).value()); // 1 unit = 3m
    mechRightElevator->SetLength(TurnToDistance(rClimbMotor.GetPosition()).value());
    mechTar->SetLength(TargetDistance.value());
}

void SubClimber::SetTarget(units::meter_t Distance) {
    TargetDistance = Distance;
}

void SubClimber::Retract() {
    SetTarget(0_m);
    lClimbMotor.SetPositionTarget(0_deg);
    rClimbMotor.SetSmartMotionTarget(20_deg);
    frc::SmartDashboard::PutNumber("Climber/test", 1);
}

void SubClimber::Extend() {
    SetTarget(1.5_m);
    lClimbMotor.SetPositionTarget(DistanceToTurn(1.5_m));
    rClimbMotor.SetPositionTarget(DistanceToTurn(1.5_m));
    frc::SmartDashboard::PutNumber("Climber/test", 2);
}

units::turn_t SubClimber::DistanceToTurn(units::meter_t distance) {
    return distance / WheelDiameter * 1_tr;
}
units::meter_t SubClimber::TurnToDistance(units::turn_t turn) {
    return turn.value() * WheelDiameter;
};