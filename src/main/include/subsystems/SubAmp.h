// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc2/command/commands.h>

#include "Constants.h"

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

  // shooter amp
  frc2::CommandPtr AmpShooter();
  frc2::CommandPtr ReverseAmpShooter();
  frc2::CommandPtr TilingRamp();

  // dizzy amp
  frc2::CommandPtr ClawOpen();
  frc2::CommandPtr ClawClose();

  frc2::CommandPtr ClawTiltDown();
  frc2::CommandPtr ClawTiltUp();
  
  
  

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  rev::CANSparkMax _ampMotorSpin{
     canid::AmpMotorSpin, rev::CANSparkMax::MotorType::kBrushless
  };

  rev::CANSparkMax _clawMotorJoint{
    canid::ClawMotorJoint, rev::CANSparkMax::MotorType::kBrushless
  };

};
