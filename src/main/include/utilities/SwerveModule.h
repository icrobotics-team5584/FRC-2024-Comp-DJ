#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <utilities/ICSparkMax.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/simulation/DCMotorSim.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/length.h>
#include <units/angular_velocity.h>
#include <memory>
#include <numbers>

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
  frc::Rotation2d GetCanCoderAngle();
  units::meters_per_second_t GetSpeed();
  units::volt_t GetDriveVoltage();
  frc::SwerveModuleState GetState();
  units::turn_t GetDriveRotations();
  units::turns_per_second_t GetDriveAngularVelocity();

 private:
  // Mechanical Constants
  static constexpr double TURNING_GEAR_RATIO = 150.0/7.0;
  static constexpr double DRIVE_GEAR_RATIO = 6.75; // L2 - Fast kit
  static constexpr units::meter_t WHEEL_RADIUS = 0.0508_m;
  static constexpr units::meter_t WHEEL_CIRCUMFERENCE = 2 * std::numbers::pi * WHEEL_RADIUS;

  // Control Gains
  // MAKE SURE TO TUNE WHILE ABOVE 12.5 VOLTS
  const double TURN_P = 3;
  const double TURN_I = 0.0;
  const double TURN_D = 0;
  const double DRIVE_P = 0.7;
  const double DRIVE_I = 0.0;
  const double DRIVE_D = 0.0;
  const double DRIVE_S = 0.070059;  // Units is V
  const double DRIVE_V = 0.7;       // Units is V/1m/s  
  const double DRIVE_A = 0;         // Units is V/1m/s^2

  // Electronics
  ctre::phoenix6::hardware::TalonFX _canDriveMotor;
  ICSparkMax _canTurnMotor;
  ctre::phoenix6::hardware::CANcoder _canTurnEncoder;

  ctre::phoenix6::configs::TalonFXConfiguration _configCanDriveMotor{};
  ctre::phoenix6::configs::TalonFXConfiguration _configCanTurnMotor{};
  ctre::phoenix6::configs::CANcoderConfiguration _configTurnEncoder{};

  // Simulation
  frc::sim::DCMotorSim _driveMotorSim{frc::DCMotor::Falcon500(), DRIVE_GEAR_RATIO, 0.05_kg_sq_m};
  frc::sim::DCMotorSim _turnMotorSim{frc::DCMotor::NEO(), TURNING_GEAR_RATIO, 0.000000001_kg_sq_m};
};
