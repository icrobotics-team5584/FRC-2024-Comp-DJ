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
#include <optional>

#include "Constants.h"
#include "utilities/ICSparkMax.h"

class SubAmp : public frc2::SubsystemBase {
 public:
  SubAmp();

  //Instance
  static SubAmp &GetInstance(){
    static SubAmp inst;
    return inst;
  }

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic() override;

  // shooter amp
  frc2::CommandPtr AmpShooter();
  frc2::CommandPtr ReverseAmpShooter();

  frc2::CommandPtr MotorTiltToAngle(units::degree_t targetAngle); // dizzy
  frc2::CommandPtr TiltArmToAngle(units::degree_t targetAngle); // arm

 private:
  // motors
  ICSparkMax _ampMotorSpin{canid::AmpMotorSpin}; // Amp shooter

  ICSparkMax _clawMotorJoint{canid::ClawMotorJoint}; // Dizzy Amp
  ICSparkMax _elevatorMotor{canid::ElevatorMotor}; // Dizzy Amp

  ICSparkMax _armMotor{canid::ArmMotor}; // arm
  ICSparkMax _armMotorFollow{canid::ArmMotorFollow}; // arm

  // encoders
  rev::SparkAbsoluteEncoder _armEncoder{_armMotor.GetAbsoluteEncoder(
      rev::SparkAbsoluteEncoder::Type::kDutyCycle)};

  // plank + claw (tune values for robot)
  static constexpr double CLAW_P = 0.0;
  static constexpr double CLAW_I = 0.0;
  static constexpr double CLAW_D = 0.0;
  static constexpr double CLAW_F = 50.0; 
  
  static constexpr double CLAW_GEAR_RATIO = 218.27;
  static constexpr units::degrees_per_second_t CLAW_MAX_VEL = 18_deg_per_s;
  static constexpr units::degrees_per_second_squared_t CLAW_MAX_ACCEL = 90_deg_per_s_sq;
  static constexpr units::degree_t CLAW_TOLERANCE = 0.5_deg; 
  static constexpr units::meter_t CLAW_LENGTH = 0.9_m;
  static constexpr units::kilogram_t CLAW_MASS = 1_kg; // only sim
  static constexpr units::degree_t CLAW_MIN_ANGLE = -180_deg; // only sim
  static constexpr units::degree_t CLAW_MAX_ANGLE = 180_deg; // only sim

  // simulating claw in smartdashboard
  frc::sim::SingleJointedArmSim _clawSim{
    frc::DCMotor::NEO(2),
    CLAW_GEAR_RATIO, 
    frc::sim::SingleJointedArmSim::EstimateMOI(CLAW_LENGTH, CLAW_MASS),
    CLAW_LENGTH,
    CLAW_MIN_ANGLE,
    CLAW_MAX_ANGLE,
    false,
    0_deg
  };

  // displaying claw in smartdashboard
  frc::Mechanism2d _doubleJointedClawMech{3, 3}; //canvas width and height
  frc::MechanismRoot2d* _clawRoot = _doubleJointedClawMech.GetRoot("clawRoot", 1, 1); //root x and y
  frc::MechanismLigament2d* _claw1Ligament = _clawRoot->Append<frc::MechanismLigament2d>("ligament1", CLAW_LENGTH.value(), 0_deg);

  nt::GenericEntry* _clawXOffset;
  nt::GenericEntry* _clawYOffset;

  // arm (tune values for robot)
  static constexpr double ARM_P = 0.0;
  static constexpr double ARM_I = 0.0;
  static constexpr double ARM_D = 0.0;
  static constexpr double ARM_F = 50.0; 
  
  static constexpr double ARM_GEAR_RATIO = 218.27;
  static constexpr units::degrees_per_second_t ARM_MAX_VEL = 18_deg_per_s;
  static constexpr units::degrees_per_second_squared_t ARM_MAX_ACCEL = 90_deg_per_s_sq;
  static constexpr units::degree_t ARM_TOLERANCE = 0.5_deg; 
  static constexpr units::meter_t ARM_LENGTH = 0.9_m;
  static constexpr units::kilogram_t ARM_MASS = 1_kg; // only sim
  static constexpr units::degree_t ARM_MIN_ANGLE = -180_deg; // only sim
  static constexpr units::degree_t ARM_MAX_ANGLE = 180_deg; // only sim

  // simulating arm in smartdashboard
  frc::sim::SingleJointedArmSim _armSim{
    frc::DCMotor::NEO(2),
    ARM_GEAR_RATIO, 
    frc::sim::SingleJointedArmSim::EstimateMOI(ARM_LENGTH, ARM_MASS),
    ARM_LENGTH,
    ARM_MIN_ANGLE,
    ARM_MAX_ANGLE,
    false,
    0_deg
  };

  // displaying arm in smartdashboard
  frc::Mechanism2d _doubleJointedArmMech{3, 3}; //canvas width and height
  frc::MechanismRoot2d* _armRoot = _doubleJointedArmMech.GetRoot("armRoot", 1, 1); //root x and y
  frc::MechanismLigament2d* _arm1Ligament = _armRoot->Append<frc::MechanismLigament2d>("ligament2", ARM_LENGTH.value(), 0_deg);

  nt::GenericEntry* _armXOffset;
  nt::GenericEntry* _armYOffset;

};
