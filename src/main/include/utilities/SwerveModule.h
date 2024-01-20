// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

// #include <ctre/Phoenix.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/length.h>
#include <units/angular_velocity.h>
#include <frc/geometry/Rotation2d.h>
#include <memory>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <numbers>
#include <utilities/ICSparkMax.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/simulation/DCMotorSim.h>


class SwerveModule {
 public:
  SwerveModule(int canDriveMotorID, int canTurnMotorID, int canTurnEncoderID, double cancoderMagOffset); 
  void SetDesiredState(const frc::SwerveModuleState& state);
  void SyncSensors();
  void SendSensorsToDash();
  void SetDesiredAngle(units::degree_t angle);
  void SetDesiredVelocity(units::meters_per_second_t velocity);
  void DriveStraightVolts(units::volt_t volts);
  void StopMotors();
  void UpdateSim(units::second_t deltaTime);
  void SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue mode);
  frc::SwerveModulePosition GetPosition();
  frc::Rotation2d GetAngle();
  units::meters_per_second_t GetSpeed();
  frc::SwerveModuleState GetState();
  units::volt_t GetDriveVoltage();

 private:
  const double TURNING_GEAR_RATIO = 150.0/7.0;
  const double DRIVE_GEAR_RATIO = 6.75; // L2 - Fast kit
  const units::meter_t WHEEL_RADIUS = 0.0508_m;
  const units::meter_t WHEEL_CIRCUMFERENCE = 2 * std::numbers::pi * WHEEL_RADIUS;

  const double TURN_P = 3;
  const double TURN_I = 0.0;
  const double TURN_D = 0;
  const double DRIVE_P = 0.000031489;
  const double DRIVE_I = 0.0;
  const double DRIVE_D = 0.0;
  const double DRIVE_F = 0;

  ctre::phoenix6::hardware::TalonFX _canDriveMotor;
  ICSparkMax _canTurnMotor;
  ctre::phoenix6::hardware::CANcoder _canTurnEncoder;

  ctre::phoenix6::configs::TalonFXConfiguration _configCanDriveMotor{};
  ctre::phoenix6::configs::TalonFXConfiguration _configCanTurnMotor{};
  ctre::phoenix6::configs::CANcoderConfiguration _configTurnEncoder{};

  frc::sim::DCMotorSim _driveMotorSim{frc::DCMotor::Falcon500(), DRIVE_GEAR_RATIO, 0.05_kg_sq_m};
  frc::sim::DCMotorSim _turnMotorSim{frc::DCMotor::NEO(), TURNING_GEAR_RATIO, 0.000000001_kg_sq_m};
};
