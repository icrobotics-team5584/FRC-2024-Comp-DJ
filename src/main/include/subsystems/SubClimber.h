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
#include <rev/CANSparkMax.h>
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

  //Periodic
  void Periodic() override;
  void SimulationPeriodic() override;

  //Sim
  void SetTarget(units::meter_t);

  //Units translation
  units::turn_t DistanceToTurn(units::meter_t distance);
  units::meter_t TurnToDistance(units::turn_t turn);

  // Actions
  void Extend();
  void Retract();
  void UpliftMiddle();

 private:
  units::meter_t TargetDistance;

  // Motor
  ICSparkMax lClimbMotor{41};
  ICSparkMax rClimbMotor{42};

  // Motor Setup
  static constexpr double GearRatio = 100;
  static constexpr double lP = 0, lI = 0.0, lD = 0.0, lF = 12.5,
                          rP = 0, rI = 0.0, rD = 0.0, rF = 12.5;
  
  static constexpr units::degrees_per_second_t MaxVelocity = 320_deg_per_s;
  static constexpr units::degrees_per_second_squared_t MaxAcceleration = 160_deg_per_s_sq;
  static constexpr units::degree_t Tolerance = 0.5_deg;

  // Unit translation
  static constexpr units::meter_t WheelCir = 0.3_m;
  
  // Sim
  static constexpr units::kilogram_square_meter_t Turret_moi = 0.005_kg_sq_m;
  frc::sim::DCMotorSim lSim{frc::DCMotor::NEO(), GearRatio, Turret_moi};
  frc::sim::DCMotorSim rSim{frc::DCMotor::NEO(), GearRatio, Turret_moi};

  frc::sim::ElevatorSim lElvSim{frc::DCMotor::NEO(), GearRatio, 0.5_kg, 0.05_m, 0_m, 1.5_m, false, 0_m};
  frc::sim::ElevatorSim rElvSim{frc::DCMotor::NEO(), GearRatio, 0.5_kg, 0.05_m, 0_m, 1.5_m, false, 0_m};

  frc::Mechanism2d mech{4,4};
  frc::MechanismRoot2d* mechRootL = mech.GetRoot("ClimberL", 1, 1);
  frc::MechanismRoot2d* mechRootR = mech.GetRoot("ClimberR", 3, 1);
  frc::MechanismRoot2d* mechRootT = mech.GetRoot("ClimberT", 2, 1);
  frc::MechanismLigament2d* mechLeftElevator = mechRootL->Append<frc::MechanismLigament2d>("Left elevator", 1, 90_deg);
  frc::MechanismLigament2d* mechRightElevator = mechRootR->Append<frc::MechanismLigament2d>("Right elevator", 3, 90_deg);
  frc::MechanismLigament2d* mechTar = mechRootT->Append<frc::MechanismLigament2d>("Target", 2, 90_deg);

  int currentPosition = 0;
};
