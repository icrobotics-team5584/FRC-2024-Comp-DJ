#pragma once
#include <frc2/command/commands.h>
#include <frc2/command/button/CommandXboxController.h>

namespace cmd {
frc2::CommandPtr ArmToAmpPos();
frc2::CommandPtr SequenceArmToAmpPos();
frc2::CommandPtr ArmToTrapPos();
frc2::CommandPtr SequenceArmToTrapPos();
frc2::CommandPtr ArmToStow();
frc2::CommandPtr IntakefullSequence();

frc2::CommandPtr AutoShootFullSequence();
frc2::CommandPtr ShootSequence();
frc2::CommandPtr IntakeSequence();
frc2::CommandPtr StartTrapSequence();
frc2::CommandPtr EndTrapSequence();
frc2::CommandPtr ShootFullSequence();
frc2::CommandPtr OuttakeNote();
frc2::CommandPtr ShootFullSequenceWithVision(frc2::CommandXboxController& controller);
frc2::CommandPtr ShootFullSequenceWithoutVision();
frc2::CommandPtr ShootSpeakerOrAmp();
frc2::CommandPtr FeedNoteToShooter();
frc2::CommandPtr PrepareToShoot();
frc2::CommandPtr ShootIntoAmp();
}  // namespace cmd
