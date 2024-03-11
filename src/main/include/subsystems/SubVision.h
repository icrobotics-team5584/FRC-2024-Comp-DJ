// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <photon/PhotonCamera.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>

#include <photon/PhotonPoseEstimator.h>
#include <photon/simulation/SimPhotonCamera.h>
#include <photon/simulation/SimVisionSystem.h>
#include <photon/simulation/SimVisionTarget.h>

#include <map>
#include <array>

class SubVision : public frc2::SubsystemBase {
 public:
  SubVision();
  static SubVision& GetInstance() {
    static SubVision inst;
    return inst;
  }

  // Will be called periodically whenever the CommandScheduler runs.

  enum FieldElement { SPEAKER, AMP, SPEAKER_SIDE, SOURCE_LEFT, SOURCE_RIGHT };

  void Periodic() override;
  void SimulationPeriodic() override;
  bool VisionHasTargets();
  bool IsOnTarget(FieldElement chosenFieldElement);
  units::degree_t getTrapAngle();
  std::optional<units::degree_t> getCamToTrapYaw();

  int FindID(FieldElement chosenFieldElement);
  std::optional<units::degree_t> GetSpecificTagYaw(FieldElement chosenFieldElement);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  photon::PhotonCamera _camera{"photonvision_5584"};
  double _bestYaw;

  std::map<FieldElement, int> blueFieldElement = {
      {SPEAKER, 7}, {SPEAKER_SIDE, 8}, {AMP, 6}, {SOURCE_LEFT, 2}, {SOURCE_RIGHT, 1}};
  std::map<FieldElement, int> redFieldElement = {
      {SPEAKER, 4}, {SPEAKER_SIDE, 3}, {AMP, 5}, {SOURCE_LEFT, 10}, {SOURCE_RIGHT, 9}};
  std::map<int, units::degree_t> trapAngle{
      {11, 116_deg}, {12, 244_deg}, {13, 0_deg},
      {14, 0_deg},   {15, 116_deg}, {16, 244_deg}};  // 11-13 is red, 14-16 is blue
  std::array<int, 3> redTrap = {11, 12, 13};
  std::array<int, 3> blueTrap = {14, 15, 16};

  photon::PhotonTrackedTarget _lastSeenTrapTag;

  frc::Transform3d _camToBot{{0_mm, 0_mm, 0_mm}, {0_deg, 0_deg, 180_deg}};

  frc::AprilTagFieldLayout _tagLayout =
      frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo);

  photon::SimVisionSystem _visionSim{
      "photonvision_5584", 45_deg, _camToBot, 9000_m, 1920, 1080, 0.0001};
};
