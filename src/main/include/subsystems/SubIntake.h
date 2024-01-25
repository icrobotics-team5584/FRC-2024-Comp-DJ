// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc2/command/commands.h>
#include <frc/DoubleSolenoid.h>
#include <frc/DigitalInput.h>

#include "Constants.h"

class SubIntake : public frc2::SubsystemBase {
 public:
  SubIntake();

  static SubIntake &GetInstance(){
    static SubIntake inst;
    return inst;
  }
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  frc2::CommandPtr IntakeNote();
  frc2::CommandPtr ExtendIntake();
  frc2::CommandPtr SpinIntake();
  bool GetIntakeState();
  void StopSpinningIntake();
  void RetractIntake();

 private:

 rev::CANSparkMax _intakeMotorSpin{
     canid::IntakeMotor, rev::CANSparkMax::MotorType::kBrushless
  };
 frc::DoubleSolenoid solIntake{pcm0::Pcm0Id, frc::PneumaticsModuleType::CTREPCM, pcm0::IntakeExtend, pcm0::IntakeRetract};

 frc::DigitalInput _intakeRetractedReed{dio::IntakeRetractedReed};
 frc::DigitalInput _intakeExtendededReed{dio::IntakeExtendedReed};
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
