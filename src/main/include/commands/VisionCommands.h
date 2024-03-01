#pragma once

#include <frc2/command/commands.h>
#include <frc2/command/button/CommandXboxController.h>

namespace cmd{
    frc2::CommandPtr VisionRotateToZero(frc2::CommandXboxController& controller); // zero yaw for april tag
    //frc2::CommandPtr VisionClimb(); 
    frc2::CommandPtr ShootSequence(frc2::CommandXboxController& controller);
} //namespace cmd