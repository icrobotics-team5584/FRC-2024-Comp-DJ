#pragma once

#include "commands/UniversalCommands.h"
#include "subsystems/SubAmp.h"
#include "subsystems/SubClimber.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubIntake.h"
#include "subsystems/SubShooter.h"

namespace cmd {
    using namespace frc2::cmd;

    frc2::CommandPtr ArmToPos(){
        // extend intake, get intake state, check arm is at home, check arm has game piece, move arm to amp pos, put to shuffleboard green box
        return RunOnce( [](){ 
            
            SubIntake::GetInstance().ExtendIntake();},
            {&SubIntake::GetInstance()}).AndThen( [](){return SubIntake::GetInstance().GetIntakeState();});
    }
}

/*
frc2::CommandPtr ArmPickUp(){
    return RunOnce(
        [](){ SubArm::GetInstance().DriveTo(0.2020_tr, -0.389_tr); }, 
        {&SubArm::GetInstance()})
    .AndThen(WaitUntil( []() { return SubArm::GetInstance().CheckPosition(); }));
    }
*/

