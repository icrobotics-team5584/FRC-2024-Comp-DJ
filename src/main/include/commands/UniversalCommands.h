#pragma once
#include <frc2/command/commands.h>

namespace cmd {
frc2::CommandPtr ArmToAmpPos();
frc2::CommandPtr SequenceArmToAmpPos();
frc2::CommandPtr ArmToTrapPos();
frc2::CommandPtr SequenceArmToTrapPos();
frc2::CommandPtr ArmToStow();
frc2::CommandPtr IntakefullSequence();

frc2::CommandPtr ShootSequence();
frc2::CommandPtr IntakeSequence();
frc2::CommandPtr TrapSequence();
frc2::CommandPtr ShootFullSequenceWithVision();
frc2::CommandPtr ShootFullSequenceWithoutVision();
frc2::CommandPtr FeedNoteToShooter();
frc2::CommandPtr PrepareToShoot();
}  // namespace cmd
