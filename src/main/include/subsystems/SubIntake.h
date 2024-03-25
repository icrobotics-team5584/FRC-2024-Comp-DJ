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

  //Define commands for intake
  frc2::CommandPtr ExtendIntake();
  frc2::CommandPtr ToggleExtendIntake();
  frc2::CommandPtr StopSpinningIntake();
  frc2::CommandPtr StartSpinningIntake();
  frc2::CommandPtr Intake();
  frc2::CommandPtr Outtake();
  frc2::CommandPtr EndIntake();
  frc2::CommandPtr IntakeSequence();
  frc2::CommandPtr CommandRetractIntake();

  //Defind functions for intake
  void Periodic() override;
  bool IsIntakeDeployed();
  void FuncRetractIntake();
  void SimulationPeriodic() override;

 private:

  //Create intake motor
  rev::CANSparkMax _intakeMotorSpin{canid::IntakeMotor, rev::CANSparkMax::MotorType::kBrushless};

  //Unused
  rev::SparkRelativeEncoder encoder{_intakeMotorSpin.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)};

  //Create intake solenoid
  frc::DoubleSolenoid solIntake{pcm1::Pcm1Id, frc::PneumaticsModuleType::REVPH,
                                pcm1::IntakeExtend, pcm1::IntakeRetract};

  //Create the intake reed switch
  frc::DigitalInput _intakeExtendedReed{dio::IntakeExtendedReed};

  //Create the intake reed switch for sim
  frc::sim::DIOSim _simIntakeExtendedReed{_intakeExtendedReed};
  
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
