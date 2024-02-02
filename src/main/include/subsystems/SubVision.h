// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <photon/PhotonCamera.h>
#include <frc/apriltag/AprilTagFieldLayout.h>

class SubVision : public frc2::SubsystemBase {
 public:
  SubVision();

  static SubVision &GetInstance(){
    static SubVision inst;
    return inst;
  }

  //Will be called periodically whenever the CommandScheduler runs.
  void Periodic() override;
  units::degree_t GetSpecificTagYaw(int correctApriltagID);

  bool VisionHasTargets();
  bool IsOnTarget();

  int FindID();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  photon::PhotonCamera _camera{"photonvision_5584"};
  double _bestYaw;

  static constexpr int BLUE_SPEAKER = 7;
  static constexpr int BLUE_AMP = 6;

  static constexpr int RED_SPEAKER = 4;
  static constexpr int RED_AMP = 5;

};
