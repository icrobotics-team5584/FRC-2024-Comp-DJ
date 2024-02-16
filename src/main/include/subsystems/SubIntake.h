// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc2/command/commands.h>
#include <frc/DoubleSolenoid.h>
#include <frc/DigitalInput.h>
#include <frc/simulation/DIOSim.h>

#include "Constants.h"

class SubIntake : public frc2::SubsystemBase {
 public:
  SubIntake();

  static SubIntake& GetInstance() {
    static SubIntake inst;
    return inst;
  }
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  frc2::CommandPtr ExtendIntake();
  bool IsIntakeAt(frc::DoubleSolenoid::Value target);
  frc2::CommandPtr StopSpinningIntake();
  frc2::CommandPtr StartSpinningIntake();
  void FuncRetractIntake();
  frc2::CommandPtr Intake();
  frc2::CommandPtr EndIntake();
  frc2::CommandPtr IntakeSequence();
  frc2::CommandPtr CommandRetractIntake();

  void SimulationPeriodic() override;

 private:
  rev::CANSparkMax _intakeMotorSpin{canid::IntakeMotor, rev::CANSparkMax::MotorType::kBrushless};
  frc::DoubleSolenoid solIntake{pcm1::Pcm1Id, frc::PneumaticsModuleType::REVPH,
                                pcm1::IntakeExtend, pcm1::IntakeRetract};

  frc::DigitalInput _intakeRetractedReed{dio::IntakeRetractedReed};
  frc::DigitalInput _intakeExtendedReed{dio::IntakeExtendedReed};
  frc::sim::DIOSim _simIntakeRetractedReed{_intakeRetractedReed};
  frc::sim::DIOSim _simIntakeExtendedReed{_intakeExtendedReed};
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
