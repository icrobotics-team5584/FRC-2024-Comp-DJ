#pragma once

#include <frc2/command/commands.h>
#include <frc2/command/button/CommandXboxController.h>

namespace cmd {
    frc2::CommandPtr ShootSequence(frc2::CommandXboxController& controller);
    frc2::CommandPtr VisionRotateToTrap();
    frc2::CommandPtr VisionTranslateToTrap();
    frc2::CommandPtr VisionAlignToSpeaker(frc2::CommandXboxController& controller);  // zero yaw for april tag
    frc2::CommandPtr VisionAlignToAmp(frc2::CommandXboxController& controller);
    frc2::CommandPtr VisionAlignToClimb();
    
}  // namespace cmd