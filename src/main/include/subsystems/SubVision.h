// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <photon/PhotonCamera.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <map>

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
  bool VisionHasTargets();
  bool IsOnTarget(FieldElement chosenFieldElement);

  int FindID(FieldElement chosenFieldElement);
  units::degree_t GetSpecificTagYaw(FieldElement chosenFieldElement);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  photon::PhotonCamera _camera{"photonvision_5584"};
  double _bestYaw;

  std::map<FieldElement, int> blueFieldElement = {
      {SPEAKER, 7}, {SPEAKER_SIDE, 8}, {AMP, 6}, {SOURCE_LEFT, 2}, {SOURCE_RIGHT, 1}};

  std::map<FieldElement, int> redFieldElement = {{SPEAKER, 0},  // change later to 4
                                                 {SPEAKER_SIDE, 3},
                                                 {AMP, 5},
                                                 {SOURCE_LEFT, 10},
                                                 {SOURCE_RIGHT, 9}};
};
