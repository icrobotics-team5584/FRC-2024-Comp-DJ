// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc2/command/commands.h>
#include <frc/DoubleSolenoid.h>
#include "utilities/ICSparkMax.h"

#include <frc/simulation/DCMotorSim.h>
#include <units/moment_of_inertia.h>

#include "Constants.h"

class SubShooter : public frc2::SubsystemBase {
 public:
  SubShooter();

  static SubShooter& GetInstance() {
    static SubShooter inst;
    return inst;
  }
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic() override;
  frc2::CommandPtr StartShooter();
  frc2::CommandPtr ShooterChangePosFar();
  frc2::CommandPtr ShooterChangePosClose();
  frc2::CommandPtr ShootNote();
  frc2::CommandPtr ShootSequence();
  frc2::CommandPtr StopShooterCommand();
  void StopShooterFunc();
  bool CheckShooterSpeed();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  static constexpr double ShooterP = 1;
  static constexpr double ShooterI = 0;
  static constexpr double ShooterD = 0;
  static constexpr double ShooterFF = 0.125;

  int ShootFarTargetRPM = 3405;
  int ShootCloseTargetRPM = 2270;

  ICSparkMax _shooterMotorMain{canid::ShooterMotorMain};
  ICSparkMax _secondaryShooterMotor{canid::SecondaryShooterMotor};

  ICSparkMax _shooterFeederMotor{canid::ShooterFeederMotor};
  frc::DoubleSolenoid solShooter{pcm0::Pcm0Id, frc::PneumaticsModuleType::CTREPCM, pcm0::ShootFar,
                                 pcm0::ShootClose};

  double mainMotorPower = 0.3;
  double secondaryMotorPower = 0.3;

  //Sim configs
  frc::sim::DCMotorSim _topShooterSim{frc::DCMotor::NEO(), 1, 0.001_kg_sq_m};
  frc::sim::DCMotorSim _bottomShooterSim{frc::DCMotor::NEO(), 1, 0.001_kg_sq_m};
  frc::sim::DCMotorSim _feederSim{frc::DCMotor::NEO550(), 1, 0.00001_kg_sq_m};
};
