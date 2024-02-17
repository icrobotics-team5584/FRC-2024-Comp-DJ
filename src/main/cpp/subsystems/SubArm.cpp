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
  _armMotor.SetInverted(true);
  _armMotor.SetConversionFactor(1 / ARM_GEAR_RATIO);
  _armMotor.SetPIDFF(ARM_P, ARM_I, ARM_D, ARM_F);
  _armMotor.ConfigSmartMotion(ARM_MAX_VEL, ARM_MAX_ACCEL, ARM_TOLERANCE);

}

// This method will be called once per scheduler run
void SubArm::Periodic() {
  frc::SmartDashboard::PutData("arm/Arm Mechanism Display", &_doubleJointedArmMech);
  frc::SmartDashboard::PutNumber("arm/Amp Shooter Motor: ", _ampMotor.Get());

  // angle of motor
  frc::SmartDashboard::PutData("arm/Arm tilt motor: ", (wpi::Sendable*)&_armMotor);
  frc::SmartDashboard::PutNumber("arm/Arm motor sim voltage: ", _armMotor.GetSimVoltage().value());

  _arm1Ligament->SetAngle(_armMotor.GetPosition() - 90_deg);
}

void SubArm::SimulationPeriodic() {
  _armSim.SetInputVoltage(_armMotor.GetSimVoltage());
  _armSim.Update(20_ms);

  auto armAngle = _armSim.GetAngle();
  auto armVel = _armSim.GetVelocity();
  _armMotor.UpdateSimEncoder(armAngle, armVel);
}

// Shooter Amp
frc2::CommandPtr SubArm::AmpShooter() {
  return StartEnd([this] { _ampMotor.Set(0.7); }, [this] { _ampMotor.Set(0); });
}

frc2::CommandPtr SubArm::ReverseAmpShooter() {
  return StartEnd([this] { _ampMotor.Set(-0.7); }, [this] { _ampMotor.Set(0); });
}

// arm
frc2::CommandPtr SubArm::TiltArmToAngle(units::degree_t targetAngle) {
  return Run([this, targetAngle] { _armMotor.SetSmartMotionTarget(targetAngle); }).Until([this] {
    return units::math::abs(_armMotor.GetPosError()) < 5_deg;
  });
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
// booleans

bool SubArm::CheckIfArmIsHome() {
  /*if (_armHomeSwitch.Get() == true) {
    return true;
  } else {
    return false;
  } */ /*BRING ME BACK*/
  return true;
}

bool SubArm::CheckIfArmHasGamePiece() {
  frc2::CommandXboxController lineBreakController{2};
  if (  lineBreakController.GetAButton()) { /*BRING ME BACK*/
    return true;
  } else {
    return false;
  }
}