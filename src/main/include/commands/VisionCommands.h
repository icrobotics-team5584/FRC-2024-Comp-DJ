#pragma once

#include <frc2/command/commands.h>

namespace cmd{
    frc2::CommandPtr VisionRotateToZero(); // zero yaw for april tag
    //frc2::CommandPtr VisionClimb(); 
    frc2::CommandPtr ShootSequence();
} //namespace cmd