#pragma once
#include <frc2/command/commands.h>

namespace cmd {
frc2::CommandPtr ArmToAmpPos();
frc2::CommandPtr SequenceArmToAmpPos();
frc2::CommandPtr ArmToTrapPos();
frc2::CommandPtr SequenceArmToTrapPos();
frc2::CommandPtr ArmToStow();

frc2::CommandPtr ShootFullSequence();
}  // namespace cmd
