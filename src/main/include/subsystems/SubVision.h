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
  bool VisionHasTargets();
  units::degree_t BestTargetYaw();
  bool IsOnTarget();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  photon::PhotonCamera _camera{"photonvision_5584"};
  photon::PhotonPipelineResult _result;
  photon::PhotonTrackedTarget _bestTarget;

  double _bestYaw;
  double _bestPitch;
  double _bestArea;
  double _bestSkew;

  double _yawTolerance;
  
};
