// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/angle.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>
#include <iostream>
#include <frc2/command/commands.h>
#include <frc2/command/button/Trigger.h>
#include <frc/MathUtil.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include "subsystems/SubArm.h"
#include "utilities/ICSparkMax.h"
#include "RobotContainer.h"

using namespace frc2::cmd;

SubArm::SubArm() {
  // amp shooter
  _ampMotor.RestoreFactoryDefaults();

  // arm
  //_armMotor.SetInverted(true);
  _armMotor.UseAbsoluteEncoder(OFFSET_ANGLE);
  _armMotor.SetPIDFF(ARM_P, ARM_I, ARM_D, ARM_F);
  _armMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  _ampMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  _armMotor.ConfigSmartMotion(ARM_MAX_VEL, ARM_MAX_ACCEL, ARM_TOLERANCE);
  _armMotor.SetSoftLimit(rev::CANSparkBase::SoftLimitDirection::kForward, (AMP_ANGLE+5_deg).value());
  _armMotor.SetSoftLimit(rev::CANSparkBase::SoftLimitDirection::kReverse, HOME_ANGLE.value());

  _ampMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus0, 500);
  _ampMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus1, 500);
  _ampMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus2, 500);
  _ampMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus3, 500);
  _ampMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus4, 500);
  _ampMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus5, 500);
  _ampMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus6, 500);
  frc::SmartDashboard::PutData("arm/Endeffector", (wpi::Sendable*)&_ampMotor);
}

// This method will be called once per scheduler run
void SubArm::Periodic() {
 // Update controller
  auto setpoint = _motionProfile.Calculate(
      120_ms, {_armMotor.GetPosition(), _armMotor.GetVelocity()}, {_targetAngle, 0_tps});
  auto nextSetpoint = _motionProfile.Calculate(
      200_ms, {_armMotor.GetPosition(), _armMotor.GetVelocity()}, {_targetAngle, 0_tps});
  units::turns_per_second_squared_t accel = (nextSetpoint.velocity - setpoint.velocity) / 20_ms;
  auto feedForward = _armFF.Calculate(setpoint.position-90_deg, setpoint.velocity, accel);
  _armMotor.SetPositionTarget(setpoint.position, feedForward);

  // Display info
  frc::SmartDashboard::PutNumber("arm/final target", _targetAngle.value());
  frc::SmartDashboard::PutNumber("arm/profile accel", accel.value());
  frc::SmartDashboard::PutNumber("arm/profile veloc", setpoint.velocity.value());
  frc::SmartDashboard::PutData("arm/Arm Mechanism Display", &_doubleJointedArmMech);
  frc::SmartDashboard::PutNumber("arm/Amp Shooter Motor: ", _ampMotor.Get());
  frc::SmartDashboard::PutBoolean("arm/Linebreak", _sdLineBreak.Get());
  frc::SmartDashboard::PutNumber("arm/End Effector Velocity", _ampMotor.GetVelocity().value());
  

  // angle of motor
  frc::SmartDashboard::PutData("arm/Arm tilt motor: ", (wpi::Sendable*)&_armMotor);
  frc::SmartDashboard::PutNumber("arm/Arm motor sim voltage: ", _armMotor.GetSimVoltage().value());
  _arm1Ligament->SetAngle(_armMotor.GetPosition() - 90_deg);
}

void SubArm::SimulationPeriodic() {
  // We have 0 degrees pointing down, physics sim expects it to point to the side
  _armSim.SetState(_armMotor.GetPosition()-90_deg, _armMotor.GetVelocity());

  _armSim.SetInputVoltage(_armMotor.GetSimVoltage());
  _armSim.Update(20_ms);

  auto armAngle = _armSim.GetAngle() + 90_deg; // bring the zero point back to straight down
  auto armVel = _armSim.GetVelocity();
  _armMotor.UpdateSimEncoder(armAngle, armVel);
}

// Shooter Amp
frc2::CommandPtr SubArm::AmpShooter() {
  return TiltArmToAngle(SHOOT_AMP_ANGLE).AndThen([this] { _ampMotor.Set(0.7); }).AndThen(Idle()).FinallyDo([this] { _ampMotor.Set(0); });
}

frc2::CommandPtr SubArm::FastAmpShooter() {
  return TiltArmToAngle(SHOOT_AMP_ANGLE).AndThen([this] { _ampMotor.Set(1); }).AndThen(Idle()).FinallyDo([this] { _ampMotor.Set(0); });
}

frc2::CommandPtr SubArm::TrapShooter() {
  static int runningCounter = 0;
  static int stoppedCounter = 0;
  static bool isRunning = false;
  return Run([this]{
    if(isRunning) {
      _ampMotor.Set(0.3);
      runningCounter++;
      if(runningCounter > 4) {
        runningCounter = 0;
        isRunning = false;
      }
    } else {
      _ampMotor.Set(-0.2);
      stoppedCounter++;
      if(stoppedCounter > 1) {
        stoppedCounter = 0;
        isRunning = true;
      }
    }
    ;}).FinallyDo([this]{_ampMotor.Set(0);});
}

// arm
frc2::CommandPtr SubArm::TiltArmToAngle(units::turn_t targetAngle) {
  return RunOnce([this, targetAngle] { _targetAngle = targetAngle; })
      .AndThen(WaitUntil([this, targetAngle] {
        return units::math::abs(targetAngle - _armMotor.GetPosition()) < 1_deg;
      }));
}

frc2::CommandPtr SubArm::StoreNote() {
  return TiltArmToAngle(HOME_ANGLE).AndThen(Run([this] {
                                              _ampMotor.Set(-1);
                                            }).Until([this] {
                                                return CheckIfArmHasGamePiece();
                                              }).FinallyDo([this] { _ampMotor.Set(0); }));
}

frc2::CommandPtr SubArm::FeedNote(){
  return Run([this]{_ampMotor.Set(-1);}).FinallyDo([this]{return  _ampMotor.Set(0);});
}

frc2::CommandPtr SubArm::Outtake() {
  return Run([this]{_ampMotor.Set(1);}).FinallyDo([this]{return _ampMotor.Set(0);});
}

frc2::CommandPtr SubArm::StopEndEffector() {
  return RunOnce([this]{_ampMotor.Set(0);});
}

// getters
bool SubArm::CheckIfArmIsHome() {
  return units::math::abs(_armMotor.GetPosition() - HOME_ANGLE) < 2_deg;
}

bool SubArm::CheckIfArmHasGamePiece() {
  if ( _sdLineBreak.Get() == BotVars::Choose(false, true)) {
    return true;
  } else {
    return false;
  }
}

units::degree_t SubArm::GetAngle() {
  return _armMotor.GetPosition();
}