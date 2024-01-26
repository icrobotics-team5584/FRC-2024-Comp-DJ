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

    _bestYaw = _bestTarget.GetYaw();
    _bestPitch = _bestTarget.GetPitch();
    _bestArea = _bestTarget.GetArea();
    _bestSkew = _bestTarget.GetSkew();

    frc::SmartDashboard::PutNumber("Vision/best target yaw: ", _bestYaw); 
    frc::SmartDashboard::PutNumber("Vision/best target pitch: ", _bestPitch); 
    frc::SmartDashboard::PutNumber("Vision/best target area: ", _bestArea); 
    frc::SmartDashboard::PutNumber("Vision/best target skew: ", _bestSkew); 
}

bool SubVision::VisionHasTargets(){ 
    bool targets = _result.HasTargets();
    return targets;
}

units::degree_t SubVision::BestTargetYaw(){
    return _bestYaw * 1_deg; 
}

bool SubVision::IsOnTarget(){
    if(_bestYaw > -0.5 && _bestYaw < 0.5){ return true; }
    else{ return false;}
}

/*
if(robotPitchAngle > 7_deg) {
      SubDriveBase::GetInstance().Drive(speed, 0_mps, rotSpeed, false); // 
    }
    else if (robotPitchAngle < -7_deg) {
      SubDriveBase::GetInstance().Drive(-speed, 0_mps, rotSpeed, false);
    }
    else{
      SubDriveBase::GetInstance().Drive(0_mps, 0_mps, rotSpeed, false);
    }
    });
*/