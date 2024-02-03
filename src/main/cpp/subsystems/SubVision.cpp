// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubVision.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <algorithm>
#include <iostream>
#include <map>
#include <bits/stdc++.h>

SubVision::SubVision() {}

using namespace std;

// This method will be called once per scheduler run
void SubVision::Periodic() {
  //frc::SmartDashboard::PutNumber("Vision/best target yaw: ", GetSpecificTagYaw(SPEAKER));
  frc::SmartDashboard::PutBoolean("Vision/best target has targets: ", VisionHasTargets());
}

bool SubVision::VisionHasTargets() {
  auto result = _camera.GetLatestResult();
  bool targets = result.HasTargets();
  return targets;
}

units::degree_t SubVision::GetSpecificTagYaw(FieldElement chosenFieldElement) {
  auto result = _camera.GetLatestResult();
  auto targets = result.GetTargets();

  int AprilTagID = FindID(chosenFieldElement);

  auto checkRightApriltag = [AprilTagID](photon::PhotonTrackedTarget apriltag){return apriltag.GetFiducialId() == AprilTagID;};
  auto tagResult = std::ranges::find_if(targets, checkRightApriltag);

  if(tagResult != targets.end()){
    return tagResult->GetYaw()*1_deg;
  }

  else{return 0_deg;}
}

bool SubVision::IsOnTarget(FieldElement chosenFieldElement){ return GetSpecificTagYaw(chosenFieldElement) > -0.4_deg && GetSpecificTagYaw(chosenFieldElement) < 0.4_deg; }

int SubVision::FindID(FieldElement chosenFieldElement){
  
  if(auto ally = frc::DriverStation::GetAlliance()){  
    if (ally.value() == frc::DriverStation::Alliance::kBlue) {
      return blueFieldElement[chosenFieldElement];
    }

    if(ally.value() == frc::DriverStation::Alliance::kRed){
      return redFieldElement[chosenFieldElement];
    }
  }

  return redFieldElement[chosenFieldElement];
}