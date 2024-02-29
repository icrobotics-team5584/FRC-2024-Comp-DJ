// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubVision.h"
#include "subsystems/SubDriveBase.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <algorithm>
#include <iostream>
#include <map>
#include <photon/simulation/SimVisionTarget.h>
#include <fmt/format.h>

SubVision::SubVision() {
  for (int i = 0; i <= 8; i++) {
    auto pose = _tagLayout.GetTagPose(i);
    if (pose.has_value()){
      photon::SimVisionTarget simTag{pose.value(), 8_in, 8_in, i};
      _visionSim.AddSimVisionTarget(simTag);
      SubDrivebase::GetInstance().DisplayPose(fmt::format("tag{}", i),
                                              pose.value().ToPose2d());
    }
  }
}

using namespace std;

// This method will be called once per scheduler run
void SubVision::Periodic() {
  frc::SmartDashboard::PutBoolean("Vision/best target has targets: ", VisionHasTargets());
}

void SubVision::SimulationPeriodic() {
  _visionSim.ProcessFrame(SubDrivebase::GetInstance().GetPose());
}

bool SubVision::VisionHasTargets() {
  auto result = _camera.GetLatestResult();
  bool targets = result.HasTargets();
  return targets;
}

std::optional<units::degree_t> SubVision::GetSpecificTagYaw(FieldElement chosenFieldElement) {
  auto result = _camera.GetLatestResult();
  auto targets = result.GetTargets();

  int AprilTagID = FindID(chosenFieldElement);

  auto checkRightApriltag = [AprilTagID](photon::PhotonTrackedTarget apriltag) {
    return apriltag.GetFiducialId() == AprilTagID;
  };
  auto tagResult = std::ranges::find_if(targets, checkRightApriltag);

  // returns yaw as degree value
  if (tagResult != targets.end()) {
    return tagResult->GetYaw() * -(1_deg);
  } 

  // return 0 when looses target
  else { return {}; }



}

// exists out when the range of yaw is between [-0.4, 0.4]
bool SubVision::IsOnTarget(FieldElement chosenFieldElement) {

  auto yaw = GetSpecificTagYaw(chosenFieldElement);

  if(yaw.has_value()){
    return yaw.value() > -0.4_deg && yaw.value() < 0.4_deg;
  }
  else{
    return false;
  }
  
}

int SubVision::FindID(FieldElement chosenFieldElement) {
  if (auto ally = frc::DriverStation::GetAlliance()) {
    if (ally.value() == frc::DriverStation::Alliance::kBlue) {
      return blueFieldElement[chosenFieldElement];
    }

    if (ally.value() == frc::DriverStation::Alliance::kRed) {
      return redFieldElement[chosenFieldElement];
    }
  }

  return redFieldElement[chosenFieldElement];
}