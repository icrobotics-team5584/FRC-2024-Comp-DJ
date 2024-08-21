// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/commands.h>

class SubAuto : public frc2::SubsystemBase {
 public:
  static SubAuto& GetInstance() {
    static SubAuto inst;
    return inst;
  }
  SubAuto();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  frc2::CommandPtr CloseNotesAuto();
  frc2::CommandPtr SimpleAuto();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
