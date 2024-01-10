// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubClimber.h"
#include <frc/smartdashboard/SmartDashboard.h>

SubClimber::SubClimber() = default;

// This method will be called once per scheduler run
void SubClimber::Periodic() {
    frc::SmartDashboard::PutData("Climber/Mech Display", &mech);
    
    arm->SetLength(pos);
}

void SubClimber::Up() {
    pos += 0.5;
}

void SubClimber::Down() {
    pos -= 0.5;
}
