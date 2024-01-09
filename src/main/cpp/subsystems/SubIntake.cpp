// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubIntake.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc2::cmd;

SubIntake::SubIntake() = default;

// This method will be called once per scheduler run
void SubIntake::Periodic() {
    frc::SmartDashboard::PutNumber("Intake/Intake Shooter Motor: ", _intakeMotorSpin.Get());
}

frc2::CommandPtr SubIntake::IntakeNote(){
    return StartEnd(
    [this]{_intakeMotorSpin.Set(0.1);},
    [this]{_intakeMotorSpin.Set(0);}
    );
}
