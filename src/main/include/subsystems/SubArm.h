// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc2/command/commands.h>
#include <units/mass.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/DigitalInput.h>
#include <frc/controller/ArmFeedforward.h>
#include <wpi/interpolating_map.h>
#include <networktables/NetworkTableEntry.h>
#include <frc/DigitalInput.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <optional>
#include <utilities/BotVars.h>
#include "Constants.h"
#include "utilities/ICSparkMax.h"

class SubArm : public frc2::SubsystemBase {
 public:
  SubArm();

  // variables
  static const inline units::degree_t OFFSET_ANGLE = BotVars::Choose(0.865_tr, 0.80809_tr); //zeroing procedure: Move arm to home, set offset to 0, read current pos, set offset to current angle - home pos
  static constexpr units::degree_t HOME_ANGLE = 0.098_tr; //0.108_tr
  static constexpr units::degree_t AMP_ANGLE = 0.467_tr; //0.65_tr
  static constexpr units::degree_t SHOOT_AMP_ANGLE = 0.642_tr;
  static constexpr units::degree_t TRAP_ANGLE = 0.517_tr;//0.62_tr;

  // Instance
  static SubArm& GetInstance() {
    static SubArm inst;
    return inst;
  }

  // Will be called periodically whenever the CommandScheduler runs.
  void Periodic() override;
  void SimulationPeriodic() override;

  // shooter amp
  frc2::CommandPtr AmpShooter();
  frc2::CommandPtr TrapShooter();
  frc2::CommandPtr FastAmpShooter();
  frc2::CommandPtr StoreNote();

  // amp
  frc2::CommandPtr TiltArmToAngle(units::turn_t targetAngle);

  frc2::CommandPtr CheckArmPos();
  frc2::CommandPtr Check();
  frc2::CommandPtr FeedNote();
  frc2::CommandPtr Outtake();
  bool CheckIfArmIsHome();
  bool CheckIfArmHasGamePiece();
  units::degree_t GetAngle();
  frc2::CommandPtr StopEndEffector();

  // Sysid commands
   frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction) {
    return _sysIdRoutine.Quasistatic(direction);
  }
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction) {
    return _sysIdRoutine.Dynamic(direction);
  }

 private:
  // motors
  ICSparkMax _ampMotor{canid::AmpMotor, 10_A}; // Amp shooter
  ICSparkMax _armMotor{canid::ArmMotor, 40_A}; // arm

  // arm (tune values for robot)
  static constexpr double ARM_P = 8;//44.597;
  static constexpr double ARM_I = 0;//0.0;
  static constexpr double ARM_D = 0;//7.5828;
  static constexpr double ARM_F = 0;//0.0;

  static constexpr auto ARM_S = 0.31072_V;
  static constexpr auto ARM_V = 0.1_V/1_tps;// 8.7588_V/1_tps;
  static constexpr auto ARM_G = 0.4236_V;
  static constexpr auto ARM_A = 0_V/1_tr_per_s_sq;

  frc::ArmFeedforward _armFF{ARM_S, ARM_G, ARM_V, ARM_A};

  static constexpr double ARM_GEAR_RATIO = 85;
  static constexpr units::degrees_per_second_squared_t ARM_MAX_ACCEL = 3000_deg_per_s_sq;
  static constexpr units::degrees_per_second_t ARM_MAX_VEL = 400_deg_per_s;// 400_deg_per_s;
  static constexpr units::degree_t ARM_TOLERANCE = 0.5_deg;
  static constexpr units::meter_t ARM_LENGTH = 0.9_m;
  static constexpr units::kilogram_t ARM_MASS = 1_kg;
  static constexpr units::degree_t ARM_MIN_ANGLE = HOME_ANGLE;
  static constexpr units::degree_t ARM_MAX_ANGLE = 225_deg;

  // Motion
  frc::TrapezoidProfile<units::turns> _motionProfile{{ARM_MAX_VEL, ARM_MAX_ACCEL}};
  units::turn_t _targetAngle = HOME_ANGLE;

  // simulating arm in smartdashboard
  frc::sim::SingleJointedArmSim _armSim{
    frc::DCMotor::NEO(1),
    ARM_GEAR_RATIO, 
    frc::sim::SingleJointedArmSim::EstimateMOI(ARM_LENGTH, ARM_MASS),
    ARM_LENGTH,
    ARM_MIN_ANGLE-90_deg, // our zero point is down, physics sim expects flat 
    ARM_MAX_ANGLE-90_deg,
    true,
    HOME_ANGLE-90_deg
  };

  // displaying arm in smartdashboard
  frc::Mechanism2d _doubleJointedArmMech{3, 3};  // canvas width and height
  frc::MechanismRoot2d* _armRoot = _doubleJointedArmMech.GetRoot("armRoot", 1, 1);  // root x and y
  frc::MechanismLigament2d* _arm1Ligament =
      _armRoot->Append<frc::MechanismLigament2d>("ligament2", ARM_LENGTH.value(), 0_deg);

  nt::GenericEntry* _armXOffset;
  nt::GenericEntry* _armYOffset;

  frc::DigitalInput _fdLineBreak{dio::FDLineBreak};
  frc::DigitalInput _sdLineBreak{dio::SDLineBreak};

  // Sysid
  frc2::sysid::SysIdRoutine _sysIdRoutine{
      frc2::sysid::Config{0.5_V/1_s, 2_V, std::nullopt, std::nullopt},
      frc2::sysid::Mechanism{[this](units::volt_t volts) { _armMotor.SetVoltage(volts); },
                             [this](frc::sysid::SysIdRoutineLog* log) {
                               log->Motor("arm")
                                   .voltage(_armMotor.GetAppliedOutput()*12_V)
                                   .position(_armMotor.GetPosition())
                                   .velocity(_armMotor.GetVelocity());
                             },
                             this}};
};
