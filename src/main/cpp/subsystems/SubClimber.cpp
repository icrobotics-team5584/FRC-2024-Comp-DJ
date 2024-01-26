// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubClimber.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkMaxAbsoluteEncoder.h>

SubClimber::SubClimber() {
  lClimbMotor.SetConversionFactor(1.0 / 30.0);
  lClimbMotor.SetPIDFF(lP, lI, lD, lF);
  lClimbMotor.SetInverted(true);
  lClimbMotor.ConfigSmartMotion(500_deg_per_s, 2500_deg_per_s_sq, Tolerance);
  rClimbMotor.SetConversionFactor(1 / gearRatio);
  rClimbMotor.SetPIDFF(rP, rI, rD, rF);
  rClimbMotor.ConfigSmartMotion(MaxVelocity, MaxAcceleration, Tolerance);
  rClimbMotor.SetInverted(false);

  frc::SmartDashboard::PutData("Climber/Left motor", (wpi::Sendable*)&lClimbMotor);
  frc::SmartDashboard::PutData("Climber/Right motor", (wpi::Sendable*)&rClimbMotor);
};

void SubClimber::Periodic() {}

void SubClimber::SimulationPeriodic() {
  frc::SmartDashboard::PutData("Climber/Mech Display", &mech);
  frc::SmartDashboard::PutNumber("Climber/Left distance",
                                 TurnToDistance(lClimbMotor.GetPosition()).value());
  frc::SmartDashboard::PutNumber("Climber/Right distance",
                                 TurnToDistance(rClimbMotor.GetPosition()).value());
  frc::SmartDashboard::PutNumber("Climber/Target distance", TargetDistance.value());

  frc::SmartDashboard::PutNumber("Climber/Elv sim position", lElvSim.GetPosition().value());
  frc::SmartDashboard::PutNumber("Climber/Elv sim velocity", lElvSim.GetVelocity().value());

  // auto volts = lClimbMotor.GetSimVoltage();
  // lSim.SetInputVoltage(volts);
  // lSim.Update(20_ms);
  // auto angle = lSim.GetAngularPosition();
  // auto velocity = lSim.GetAngularVelocity();
  // lClimbMotor.UpdateSimEncoder(angle,velocity);

  // auto volts = lClimbMotor.GetSimVoltage();
  // lElvSim.SetInputVoltage(volts);
  // lElvSim.Update(20_ms);
  // auto angle = DistanceToTurn(lElvSim.GetPosition());
  // auto velocity = DistanceToTurn(lElvSim.GetVelocity());
  // lClimbMotor.UpdateSimEncoder(angle,velocity);

  lElvSim.SetInputVoltage(lClimbMotor.GetSimVoltage());
  lElvSim.Update(20_ms);
  lClimbMotor.UpdateSimEncoder(DistanceToTurn(lElvSim.GetPosition()),
                               DistanceToTurn(lElvSim.GetVelocity()));

  auto volts = rClimbMotor.GetSimVoltage();
  rSim.SetInputVoltage(volts);
  rSim.Update(20_ms);
  auto angle = rSim.GetAngularPosition();
  auto velocity = rSim.GetAngularVelocity();
  rClimbMotor.UpdateSimEncoder(angle, velocity);

  mechLeftElevator->SetLength(TurnToDistance(lClimbMotor.GetPosition()).value() +
                              BaseHeight.value());
  mechRightElevator->SetLength(TurnToDistance(rClimbMotor.GetPosition()).value() +
                               BaseHeight.value());
  mechTar->SetLength(TargetDistance.value());
}

void SubClimber::SetTarget(units::meter_t Distance) {
  TargetDistance = Distance;
}

units::turn_t SubClimber::DistanceToTurn(units::meter_t distance) {
  return distance / WheelCir * 1_tr;
}

units::radians_per_second_t SubClimber::DistanceToTurn(units::meters_per_second_t distance) {
  return distance / WheelCir * 1_rad;
}

units::meter_t SubClimber::TurnToDistance(units::turn_t turn) {
  return turn.value() * WheelCir;
};

void SubClimber::DriveToDistance(units::meter_t distance) {
  SetTarget(distance);
  lClimbMotor.SetSmartMotionTarget(DistanceToTurn(distance - BaseHeight));
  rClimbMotor.SetSmartMotionTarget(DistanceToTurn(distance - BaseHeight));
}

void SubClimber::Retract() {
  DriveToDistance(BaseHeight);
  // Start(-1);
}

void SubClimber::Extend() {
  DriveToDistance(1.3_m);
  // Start(1);
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
  return RunOnce([] { SubClimber::GetInstance().Extend(); });
}

frc2::CommandPtr SubClimber::ClimberRetract() {
  return RunOnce([] { SubClimber::GetInstance().Retract(); });
}

frc2::CommandPtr SubClimber::ClimberExtendManual() {
  return RunOnce([] { SubClimber::GetInstance().Start(0.5); });
}

frc2::CommandPtr SubClimber::ClimberRetractManual() {
  return RunOnce([] { SubClimber::GetInstance().Start(-0.5); });
}

frc2::CommandPtr SubClimber::ClimberStop() {
  return RunOnce([] { SubClimber::GetInstance().Stop(); });
}