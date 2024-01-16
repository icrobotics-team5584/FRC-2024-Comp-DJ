// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/simulation/ElevatorSim.h>
#include <frc/system/plant/DCMotor.h>
#include <rev/SparkMaxAbsoluteEncoder.h>
#include <Constants.h>

#include "Utilities/ICSparkMax.h"

#include <frc/simulation/DCMotorSim.h>
#include <units/angle.h>
#include <units/moment_of_inertia.h>

class SubClimber : public frc2::SubsystemBase {
 public:
  SubClimber();

  static SubClimber &GetInstance(){
    static SubClimber inst;
    return inst;
  }

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic() override;

  void SetTarget(units::meter_t);

  units::turn_t DistanceToTurn(units::meter_t distance);
  units::meter_t TurnToDistance(units::turn_t turn);

  void Extend();
  void Retract();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  units::meter_t TargetDistance;

  ICSparkMax lClimbMotor{41};
  ICSparkMax rClimbMotor{42};

  // rev::SparkAbsoluteEncoder lClimbEncoder{lClimbMotor.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle)};
  // rev::SparkAbsoluteEncoder rClimbEncoder{rClimbMotor.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle)};

  static constexpr double GearRatio = 100;
  static constexpr double lP = 0.1, lI = 0.0, lD = 0.1, lF = 0.0,
                          rP = 0.1, rI = 0.0, rD = 0.1, rF = 0.0;
  
  static constexpr units::degrees_per_second_t MaxVelocity = 6000_deg_per_s;
  static constexpr units::degrees_per_second_squared_t MaxAcceleration = 6000_deg_per_s_sq;
  static constexpr units::degree_t Tolerance = 30_deg;

  static constexpr units::meter_t WheelDiameter = 0.12_m;

  static constexpr units::kilogram_square_meter_t Turret_moi = 0.005_kg_sq_m;
  frc::sim::DCMotorSim lSim{frc::DCMotor::NEO(), GearRatio, Turret_moi};
  frc::sim::DCMotorSim rSim{frc::DCMotor::NEO(), GearRatio, Turret_moi};

  frc::Mechanism2d mech{4,4};
  frc::MechanismRoot2d* mechRootL = mech.GetRoot("ClimberL", 1, 1);
  frc::MechanismRoot2d* mechRootR = mech.GetRoot("ClimberR", 3, 1);
  frc::MechanismRoot2d* mechRootT = mech.GetRoot("ClimberT", 2, 1);
  frc::MechanismLigament2d* mechLeftElevator = mechRootL->Append<frc::MechanismLigament2d>("Left elevator", 1, 90_deg);
  frc::MechanismLigament2d* mechRightElevator = mechRootR->Append<frc::MechanismLigament2d>("Right elevator", 3, 90_deg);
  frc::MechanismLigament2d* mechTar = mechRootT->Append<frc::MechanismLigament2d>("Target", 2, 90_deg);
};
