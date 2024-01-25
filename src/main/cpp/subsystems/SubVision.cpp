// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubVision.h"
#include <frc/smartdashboard/SmartDashboard.h>

SubVision::SubVision(){}

// This method will be called once per scheduler run
void SubVision::Periodic(){
    _result = _camera.GetLatestResult();
    _bestTarget = _result.GetBestTarget();

    double yaw = _bestTarget.GetYaw();
    double pitch = _bestTarget.GetPitch();
    double area = _bestTarget.GetArea();
    double skew = _bestTarget.GetSkew();

    frc::SmartDashboard::PutNumber("Vision/bestTarget yaw: ", yaw); 
    frc::SmartDashboard::PutNumber("Vision/bestTarget pitch: ", pitch); 
    frc::SmartDashboard::PutNumber("Vision/bestTarget area: ", area); 
    frc::SmartDashboard::PutNumber("Vision/bestTarget skew: ", skew);

}

bool SubVision::VisionHasTargets(){ 
    bool targets = _result.HasTargets();
    return targets;
}

