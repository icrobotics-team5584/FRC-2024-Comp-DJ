// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc2/command/commands.h>

#include "Constants.h"

class SubShooter : public frc2::SubsystemBase {
 public:
  SubShooter();

  static SubShooter &GetInstance(){
    static SubShooter inst;
    return inst;
  }
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  frc2::CommandPtr ShootNote();

 private:

 rev::CANSparkMax _shooterMotorMainSpin{
     canid::ShooterMotorMain, rev::CANSparkMax::MotorType::kBrushless
  };

 rev::CANSparkMax _shooterMotorFollowerSpin{
     canid::ShooterMotorFollower, rev::CANSparkMax::MotorType::kBrushless
  };

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
