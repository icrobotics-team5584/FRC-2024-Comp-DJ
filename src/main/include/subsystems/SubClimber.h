// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <Constants.h>

#include <numbers>

#include "Utilities/ICSparkMax.h"

#include <frc/simulation/ElevatorSim.h>
#include <frc/system/plant/DCMotor.h>
#include <rev/CANSparkMax.h>
#include <frc/simulation/DCMotorSim.h>
#include <units/angle.h>
#include <units/moment_of_inertia.h>

#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>

#include <frc2/command/commands.h>

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

  //Units translation
  units::turn_t DistanceToTurn(units::meter_t distance);
  units::radians_per_second_t DistanceToTurn(units::meters_per_second_t distance);
  units::meter_t TurnToDistance(units::turn_t turn);

  //Tools
  void DriveToDistance(units::meter_t distance);

  // Actions
  void Extend();
  void Retract();
  void Start(double power);
  void Stop();

  frc2::CommandPtr ClimberExtend();
  frc2::CommandPtr ClimberRetract();
  frc2::CommandPtr ClimberStop();

 private:
  units::meter_t TargetDistance;

  // Motor
  ICSparkMax lClimbMotor{41};
  ICSparkMax rClimbMotor{42};

  // Motor Setup
  static constexpr double gearRatio = 30.0;
  static constexpr double lP = 0.1, lI = 0.0, lD = 0.0, lF = 0,
  
                          rP = 0.1, rI = 0.0, rD = 0.0, rF = 0;

  // Unit translation
  static constexpr units::meter_t WheelCir = 0.3_m;

  // Robot info
  static constexpr units::meter_t BaseHeight = 0.2_m;
  
  // Sim

  frc::sim::ElevatorSim lElvSim{frc::DCMotor::NEO(), gearRatio, 26_kg, (WheelCir/std::numbers::pi)/2, BaseHeight, 1.5_m, false, BaseHeight};
  frc::sim::ElevatorSim rElvSim{frc::DCMotor::NEO(), gearRatio, 26_kg, (WheelCir/std::numbers::pi)/2, BaseHeight, 1.5_m, false, BaseHeight};

  frc::Mechanism2d mech{4,4};
  frc::MechanismRoot2d* mechRootL = mech.GetRoot("ClimberL", 1, 1);
  frc::MechanismRoot2d* mechRootR = mech.GetRoot("ClimberR", 3, 1);
  frc::MechanismRoot2d* mechRootT = mech.GetRoot("ClimberT", 2, 1);
  frc::MechanismLigament2d* mechLeftElevator = mechRootL->Append<frc::MechanismLigament2d>("Left elevator", 1, 90_deg);
  frc::MechanismLigament2d* mechRightElevator = mechRootR->Append<frc::MechanismLigament2d>("Right elevator", 3, 90_deg);
  frc::MechanismLigament2d* mechTar = mechRootT->Append<frc::MechanismLigament2d>("Target", 2, 90_deg);

  int currentPosition = 0;
};
