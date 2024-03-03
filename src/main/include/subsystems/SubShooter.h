// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc2/command/commands.h>
#include <frc/DoubleSolenoid.h>
#include "utilities/ICSparkMax.h"
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/DigitalInput.h>

#include <frc/simulation/DCMotorSim.h>
#include <units/moment_of_inertia.h>
#include <frc/Encoder.h>
#include <units/velocity.h>
#include <frc/DigitalInput.h>
#include <frc/simulation/EncoderSim.h>

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
  frc2::CommandPtr StartFeeder();
  frc2::CommandPtr ShootSequence();
  frc2::CommandPtr AutoShootSequence();
  frc2::CommandPtr StopShooterCommand();
  frc2::CommandPtr FeedNoteToArm();
  void StopShooterFunc();
  bool CheckShooterSpeed();
  bool CheckShooterLineBreak();
  void UpdatePIDFF();
  frc2::CommandPtr StopFeeder();
  void StopFeederFunc();
  frc2::CommandPtr Outtake();
  frc2::CommandPtr StartFeederSlow();
  frc2::CommandPtr ReverseFeeder();


 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  static constexpr double ShooterP = 0.2; 
  static constexpr double ShooterI = 0;
  static constexpr double ShooterD = 0;

  units::turns_per_second_t ShootFarTarget = 60_tps;
  units::turns_per_second_t ShootCloseTarget = 60_tps;

  units::turns_per_second_t CurrentShooterTarget = 0_tps;

  static constexpr units::volt_t kS = 0.0000001_V;
  static constexpr decltype(1_V / 1_tps) kV = 0.135_V / 1_tps;
  static constexpr decltype(1_V / 1_tr_per_s_sq) kA = 0.001_V / 1_tr_per_s_sq;
  double _bottomEncoderPositionPrev = 0;
  double _topEncoderPositionPrev = 0;
  double _bottomEncoderDiff = 0;
  double _topEncoderDiff = 0;
  std::array<double, 3> _topPastVelocityMeasurements{0,0,0};
  std::array<double, 3> _bottomPastVelocityMeasurements{0,0,0};
  double _topPastVelocityAvg = 0;
  double _bottomPastVelocityAvg = 0;
  frc::SimpleMotorFeedforward<units::turns> _shooterFF{kS, kV, kA};

  frc::Encoder _topEncoder{dio::TopShooterEncoderChannelA, dio::TopShooterEncoderChannelB, false , frc::Encoder::EncodingType::k1X};
  frc::Encoder _bottomEncoder{dio::BottomShooterEncoderChannelA, dio::BottomShooterEncoderChannelB , false,   frc::Encoder::EncodingType::k1X};
  
  frc::PIDController _topPID{ShooterP, ShooterI, ShooterD};
  frc::PIDController _bottomPID{ShooterP, ShooterI, ShooterD};

  ICSparkMax _topShooterMotor{canid::TopShooterMotor, 30_A};
  ICSparkMax _bottomShooterMotor{canid::BottomShooterMotor, 30_A};

  ICSparkMax _shooterFeederMotor{canid::ShooterFeederMotor, 10_A};
  frc::DoubleSolenoid solShooter{pcm1::Pcm1Id, frc::PneumaticsModuleType::REVPH, pcm1::ShootFar,
                                 pcm1::ShootClose};

  frc::DigitalInput _shooterLineBreak{dio::ShooterLineBreak};

  //Sim configs
  frc::sim::DCMotorSim _topShooterSim{frc::DCMotor::NEO(), 1, 0.001_kg_sq_m};
  frc::sim::DCMotorSim _bottomShooterSim{frc::DCMotor::NEO(), 1, 0.001_kg_sq_m};
  frc::sim::DCMotorSim _feederSim{frc::DCMotor::NEO550(), 1, 0.00001_kg_sq_m};
  frc::sim::EncoderSim _topEncoderSim{_topEncoder};
  frc::sim::EncoderSim _bottomEncoderSim{_bottomEncoder};
};
