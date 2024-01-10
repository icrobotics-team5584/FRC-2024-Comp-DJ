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

  // dizzy amp
  frc2::CommandPtr MotorTiltToAngle(units::degree_t targetAngle);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  ICSparkMax _motorForTilt{canid::MotorForTilt};
  ICSparkMax _ampMotorSpin{canid::AmpMotorSpin};
  ICSparkMax _clawMotorJoint{canid::ClawMotorJoint};

  // tilting a motor to an angle

  rev::SparkMaxAbsoluteEncoder _topEncoder{_motorForTilt.GetAbsoluteEncoder(
      rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle)};
  
  // plank + claw
  static constexpr double P = 0.0;
  static constexpr double I = 0.0;
  static constexpr double D = 0.0;
  static constexpr double F = 15;
  
  static constexpr double GEAR_RATIO = 218.27;
  static constexpr units::degrees_per_second_t MAX_VEL = 18_deg_per_s;
  static constexpr units::degrees_per_second_squared_t MAX_ACCEL = 90_deg_per_s_sq;
  static constexpr units::degree_t TOLERANCE = 0.5_deg; 
  static constexpr units::meter_t LENGTH = 0.9_m;
  static constexpr units::kilogram_t MASS = 1_kg; // only sim
  static constexpr units::degree_t MIN_ANGLE = -180_deg; // only sim
  static constexpr units::degree_t MAX_ANGLE = 180_deg; // only sim


  // simulating claw in smartdashboard
  frc::sim::SingleJointedArmSim _clawSim{
    frc::DCMotor::NEO(2),
    GEAR_RATIO, 
    frc::sim::SingleJointedArmSim::EstimateMOI(LENGTH, MASS),
    LENGTH,
    MIN_ANGLE,
    MAX_ANGLE,
    false,
    0_deg
  };

};
