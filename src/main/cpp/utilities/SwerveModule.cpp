// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utilities/SwerveModule.h"
#include "utilities/Conversion.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>
#include <frc/RobotBase.h>


SwerveModule::SwerveModule(int canDriveMotorID, int canTurnMotorID,
                           int canTurnEncoderID, double cancoderMagOffset)
    : _canDriveMotor(canDriveMotorID, "Canivore"),
      _canTurnMotor(canTurnMotorID, "Canivore"),
      _canTurnEncoder(canTurnEncoderID, "Canivore") {
  
  using namespace ctre::phoenix6::signals;
  using namespace ctre::phoenix6::configs;

  // Config CANCoder
  _configTurnEncoder.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue::Signed_PlusMinusHalf;
  _configTurnEncoder.MagnetSensor.MagnetOffset = cancoderMagOffset;
  _canTurnEncoder.GetConfigurator().Apply(_configTurnEncoder);

  // Config Turning Motor
  _configCanTurnMotor.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
  _configCanTurnMotor.ClosedLoopGeneral.ContinuousWrap = true;
  _configCanTurnMotor.Slot0.kP = TURN_P;
  _configCanTurnMotor.Slot0.kI = TURN_I;
  _configCanTurnMotor.Slot0.kD = TURN_D;
  _configCanTurnMotor.CurrentLimits.SupplyCurrentLimitEnable = true;
  _configCanTurnMotor.CurrentLimits.SupplyCurrentLimit = 20.0;
  _configCanTurnMotor.CurrentLimits.SupplyCurrentThreshold = 40.0;
  _configCanTurnMotor.CurrentLimits.SupplyTimeThreshold = 0.5;
  _configCanTurnMotor.MotorOutput.NeutralMode = NeutralModeValue::Brake;
  _canTurnMotor.GetConfigurator().Apply(_configCanTurnMotor);

  _canTurnMotor.SetInverted(true); // make counter clockwise rotations positive

  // Config Driving Motor
  _canDriveMotor.GetConfigurator().Apply(TalonFXConfiguration{});
  _configCanDriveMotor.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue::RotorSensor;
  _configCanDriveMotor.ClosedLoopGeneral.ContinuousWrap = false;
  _configCanDriveMotor.Slot0.kP = DRIVE_P;
  _configCanDriveMotor.Slot0.kI = DRIVE_I;
  _configCanDriveMotor.Slot0.kD = DRIVE_D;
  _configCanDriveMotor.CurrentLimits.SupplyCurrentLimitEnable = true;
  _configCanDriveMotor.CurrentLimits.SupplyCurrentLimit = 20.0;
  _configCanDriveMotor.CurrentLimits.SupplyCurrentThreshold = 40.0;
  _configCanDriveMotor.CurrentLimits.SupplyTimeThreshold = 0.5;
  _configCanDriveMotor.Slot0.kS = 0.62004; // Units is V
  _configCanDriveMotor.Slot0.kV = 2.2731; // Units is V/1m/s
  // _configCanDriveMotor.Slot0.kA = 0.23244_V/1_mps_sq; // Units is V/1m/s^2
  _canDriveMotor.GetConfigurator().Apply(_configCanDriveMotor);
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& referenceState) {
  // Optimize the reference state to avoid spinning further than 90 degrees
  auto targetState = frc::SwerveModuleState::Optimize(referenceState, GetAngle());

  // Move target angle so we can cross over the 180 degree line without going the long way round
  auto difference = targetState.angle.Degrees() - GetAngle().Degrees();
  difference = frc::InputModulus(difference, -180_deg, 180_deg);
  auto targetAngle = GetAngle().Degrees() + difference;

  // Drive! These functions do some conversions and send targets to falcons
  SetDesiredAngle(targetAngle);
  SetDesiredVelocity(targetState.speed);
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
  auto wheelRot = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(_canDriveMotor.GetPosition(), _canDriveMotor.GetVelocity());
  units::meter_t distance = (WHEEL_CIRCUMFERENCE.value() * wheelRot.value()) * 1_m;
  return {distance, GetAngle()};
}

void SwerveModule::SendSensorsToDash() {
  std::string driveMotorName = "drive motor " + std::to_string(_canDriveMotor.GetDeviceID());
  std::string turnMotorName = "turn motor " + std::to_string(_canTurnMotor.GetDeviceID());
  std::string turnEncoderName = "turn encoder " + std::to_string(_canTurnEncoder.GetDeviceID());

  frc::SmartDashboard::PutNumber(driveMotorName + " Target velocity", _canDriveMotor.GetClosedLoopReference().GetValue());
  frc::SmartDashboard::PutNumber(driveMotorName + " velocity", _canDriveMotor.GetVelocity().GetValue().value());
  frc::SmartDashboard::PutNumber(turnMotorName  + " position degrees", GetAngle().Degrees().value());
  frc::SmartDashboard::PutNumber(turnMotorName  + " target", _canTurnMotor.GetClosedLoopReference().GetValue());
  frc::SmartDashboard::PutNumber(turnMotorName  + " error", _canTurnMotor.GetClosedLoopError().GetValue());
  frc::SmartDashboard::PutNumber(turnEncoderName+ " Abs position", _canTurnEncoder.GetAbsolutePosition().GetValue().value());
}

frc::Rotation2d SwerveModule::GetAngle() {
  auto angle = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(_canTurnMotor.GetPosition(), _canTurnMotor.GetVelocity());
  // frc::Rotation2d rot = angle.convert<frc::Rotation2d>();
  units::degree_t deg = angle;
  frc::Rotation2d rot = deg;
  return rot;
}

units::meters_per_second_t SwerveModule::GetSpeed() {
  // If in simulation, pretend swerve is always at target state
  return (_canDriveMotor.GetVelocity().GetValue().value() * WHEEL_CIRCUMFERENCE.value()) * 1_mps;
}

frc::SwerveModuleState SwerveModule::GetState() {
  return {GetSpeed(), GetAngle()};
}

void SwerveModule::SetDesiredAngle(units::degree_t angle) {
  _canTurnMotor.SetControl(ctre::phoenix6::controls::PositionVoltage{angle});
}

void SwerveModule::SetDesiredVelocity(units::meters_per_second_t velocity) {
  // double falconVel = Conversions::RobotVelToFalconVel(
  //     velocity, DRIVE_GEAR_RATIO, WHEEL_RADIUS);
  // units::volt_t ffvolts = _feedFoward.Calculate(velocity);
  // double ffpercent = ffvolts.value()/12;
  // _canDriveMotor.Set(TalonFXControlMode::Velocity, falconVel,
  //                    DemandType::DemandType_ArbitraryFeedForward, ffpercent);
  // _simulatorDistanceTravelled += velocity * 20_ms;

  _canDriveMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{(velocity.value()/WHEEL_CIRCUMFERENCE.value())*1_tps});
}

void SwerveModule::StopMotors() {
  _canDriveMotor.Set(0);
  _canTurnMotor.Set(0);
}

void SwerveModule::SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue mode) {
  _configCanDriveMotor.MotorOutput.NeutralMode = mode;
  _configCanTurnMotor.MotorOutput.NeutralMode = mode;
  _canTurnMotor.GetConfigurator().Apply(_configCanTurnMotor);
  _canDriveMotor.GetConfigurator().Apply(_configCanDriveMotor);
}

void SwerveModule::SyncSensors() {
  // units::degree_t cancoderDegrees = _canTurnEncoder.GetAbsolutePosition().GetValue();
  // _canTurnMotor.
}

