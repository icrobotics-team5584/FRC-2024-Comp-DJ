// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/simulation/ElevatorSim.h>
#include <rev/CANSparkMax.h>
#include <Constants.h>

#include "Utilities/ICSparkMax.h"

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

  void Up();
  void Down();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax leftClimbMotor{41, rev::CANSparkMax::MotorType::kBrushless}; //not set up yet
  rev::CANSparkMax rightClimbMotor{42, rev::CANSparkMax::MotorType::kBrushless};//not set up yet

  frc::Mechanism2d mech{3,5};
  frc::MechanismRoot2d* root = mech.GetRoot("Climber", 1, 1);
  frc::MechanismLigament2d* arm = root->Append<frc::MechanismLigament2d>("Arm", 2, 90_deg);

  float pos = 2;
};
