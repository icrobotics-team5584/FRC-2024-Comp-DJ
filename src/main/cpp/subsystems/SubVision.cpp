// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubVision.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <algorithm>

SubVision::SubVision() {}

// This method will be called once per scheduler run
void SubVision::Periodic() {
  frc::SmartDashboard::PutNumber("Vision/best target yaw: ", BestTargetYaw(0).value());
  frc::SmartDashboard::PutBoolean("Vision/best target has targets: ", VisionHasTargets());
}

bool SubVision::VisionHasTargets() {
  auto result = _camera.GetLatestResult();
  bool targets = result.HasTargets();
  return targets;
}

units::degree_t SubVision::BestTargetYaw(int correctApriltagID) {
  auto result = _camera.GetLatestResult();
  auto targets = result.GetTargets();

  auto checkRightApriltag = [correctApriltagID](photon::PhotonTrackedTarget apriltag){return apriltag.GetFiducialId() == correctApriltagID;};
  auto tagResult = std::ranges::find_if(targets, checkRightApriltag);

  if(tagResult != targets.end()){
    return tagResult->GetYaw()*1_deg;
  }

  else{return 0_deg;}
}

bool SubVision::IsOnTarget() {
  return BestTargetYaw(0) > -0.4_deg && BestTargetYaw(0) < 0.4_deg;
}