// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubIntake.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc2::cmd;

SubIntake::SubIntake(){
    frc::SmartDashboard::PutString("Intake/Intake Deploy State: ", "Intake retracted");
}
// This method will be called once per scheduler run
void SubIntake::Periodic() {
    if (solIntake.Get() == 1){
       frc::SmartDashboard::PutString("Intake/Intake Deploy State: ", "Intake deployed");
    } else if (solIntake.Get() == 2){
       frc::SmartDashboard::PutString("Intake/Intake Deploy State: ", "Intake retracted");
    }
    frc::SmartDashboard::PutNumber("Intake/Intake Shooter Motor: ", _intakeMotorSpin.Get());
}

frc2::CommandPtr SubIntake::IntakeNote(){
 return ExtendIntake().AndThen(SpinIntake()).FinallyDo([this]{StopSpinningIntake(), RetractIntake();});
}

frc2::CommandPtr SubIntake::ExtendIntake(){
 return RunOnce([this]{solIntake.Set(frc::DoubleSolenoid::kForward);}).AndThen(Wait(0.3_s));
}

frc2::CommandPtr SubIntake::SpinIntake(){
    return StartEnd([this]{_intakeMotorSpin.Set(0.1);},
                    [this]{_intakeMotorSpin.Set(0);});
}

void SubIntake::RetractIntake(){
    solIntake.Set(frc::DoubleSolenoid::kReverse);
}

void SubIntake::StopSpinningIntake(){
    _intakeMotorSpin.Set(0);
}
