// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utilities/SwerveModule.h"
#include "utilities/Conversion.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>
#include <frc/RobotBase.h>
#include <iostream>

SwerveModule::SwerveModule(int canDriveMotorID, int canTurnMotorID, int canTurnEncoderID,
                           double cancoderMagOffset)
    : _canDriveMotor(canDriveMotorID, "Canivore"),
      _canTurnMotor(canTurnMotorID, 40_A),
      _canTurnEncoder(canTurnEncoderID, "Canivore") {
  using namespace ctre::phoenix6::signals;
  using namespace ctre::phoenix6::configs;

  // Config CANCoder
  _configTurnEncoder.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue::Unsigned_0To1;
  _configTurnEncoder.MagnetSensor.SensorDirection = SensorDirectionValue::Clockwise_Positive;
  _configTurnEncoder.MagnetSensor.MagnetOffset = cancoderMagOffset;
  _canTurnEncoder.GetConfigurator().Apply(_configTurnEncoder);

  // Config Turning Motor
  _canTurnMotor.RestoreFactoryDefaults();
  _canTurnMotor.SetCANTimeout(500);
  _canTurnMotor.SetConversionFactor(1.0/TURNING_GEAR_RATIO);
  _canTurnMotor.EnableClosedLoopWrapping(0_tr, 1_tr);
  _canTurnMotor.SetPIDFF(TURN_P, TURN_I, TURN_D);
  //_canTurnMotor.SetInverted(true);
  _canTurnMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  _canTurnMotor.BurnFlash();
  _canTurnMotor.SetCANTimeout(10);

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
  _configCanDriveMotor.Slot0.kA = 0.23244; // Units is V/1m/s^2
  _canDriveMotor.GetConfigurator().Apply(_configCanDriveMotor);
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& referenceState) {
  // Optimize the reference state to avoid spinning further than 90 degrees
  auto targetState = frc::SwerveModuleState::Optimize(referenceState, GetAngle());

  // Move target angle so we can cross over the 180 degree line without going the long way round
  // auto difference = targetState.angle.Degrees() - GetAngle().Degrees();
  // difference = frc::InputModulus(difference, -180_deg, 180_deg);
  // auto targetAngle = GetAngle().Degrees() + difference;

  // Drive! These functions do some conversions and send targets to falcons
  SetDesiredAngle(targetState.angle.Degrees());
  SetDesiredVelocity(targetState.speed);
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
  auto wheelRot = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
      _canDriveMotor.GetPosition(), _canDriveMotor.GetVelocity());
  units::meter_t distance = (WHEEL_CIRCUMFERENCE.value() * wheelRot.value()) * 1_m;
  return {distance, GetAngle()};
}

void SwerveModule::SendSensorsToDash() {
  // clang-format off
  std::string driveMotorName = "swerve/drive motor " + std::to_string(_canDriveMotor.GetDeviceID());
  std::string turnMotorName = "swerve/turn motor " + std::to_string(_canTurnMotor.GetDeviceId());
  std::string turnEncoderName = "swerve/turn encoder " + std::to_string(_canTurnEncoder.GetDeviceID());

  frc::SmartDashboard::PutNumber(driveMotorName + " Target velocity", _canDriveMotor.GetClosedLoopReference().GetValue());
  frc::SmartDashboard::PutNumber(driveMotorName + " velocity", _canDriveMotor.GetVelocity().GetValue().value());
  frc::SmartDashboard::PutNumber(turnMotorName  + " position", GetAngle().Degrees().value()/360.0);
  frc::SmartDashboard::PutNumber(turnMotorName  + " target", _canTurnMotor.GetPositionTarget().value());
  frc::SmartDashboard::PutNumber(turnMotorName  + " error", _canTurnMotor.GetPosError().value());
  frc::SmartDashboard::PutNumber(turnEncoderName+ " Abs position", _canTurnEncoder.GetAbsolutePosition().GetValue().value());
  // clang-format on
}

frc::Rotation2d SwerveModule::GetAngle() {
  units::radian_t turnAngle = _canTurnMotor.GetPosition();
  return turnAngle;
}

units::meters_per_second_t SwerveModule::GetSpeed() {
  return (_canDriveMotor.GetVelocity().GetValue().value() * WHEEL_CIRCUMFERENCE.value()) * 1_mps;
}

frc::SwerveModuleState SwerveModule::GetState() {
  return {GetSpeed(), GetAngle()};
}

void SwerveModule::SetDesiredAngle(units::degree_t angle) {
  _canTurnMotor.SetPositionTarget(angle);
}

void SwerveModule::SetDesiredVelocity(units::meters_per_second_t velocity) {
  std::cout <<"SetDesiredVelocity called" << velocity.value() << '\n';
  units::turns_per_second_t TurnsPerSec = (velocity.value() / WHEEL_CIRCUMFERENCE.value())*1_tps;
   units::volt_t ffvolts = _feedFoward.Calculate(velocity);

  _canDriveMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{
      (TurnsPerSec)}.WithFeedForward(ffvolts));
}

void SwerveModule::StopMotors() {
  _canDriveMotor.Set(0);
  _canTurnMotor.Set(0);
}

void SwerveModule::SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue mode) {
  //   _configCanDriveMotor.MotorOutput.NeutralMode = mode;
  //   _canTurnMotor.
  //   _canDriveMotor.GetConfigurator().Apply(_configCanDriveMotor);
}

void SwerveModule::SyncSensors() {
  std::cout << "SyncSensors run pre anything" << '\n';
  _canTurnMotor.SetCANTimeout(500);
  units::turn_t truePos = _canTurnEncoder.GetAbsolutePosition().GetValue();
  int maxAttempts = 15;
  int currentAttempts = 0;
  units::turn_t tolerance = 0.01_tr;
  
  std::cout << "SyncSensors run pre while loop" << '\n';

  while(units::math::abs(_canTurnMotor.GetPosition()-truePos) > tolerance && currentAttempts < maxAttempts) {
    std::cout << "attempt " << currentAttempts << '\n';
   _canTurnMotor.SetPosition(truePos);
    currentAttempts++;
  }


  currentAttempts = 0;
  _canTurnMotor.SetCANTimeout(10);
}
