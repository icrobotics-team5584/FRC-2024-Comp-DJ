#pragma once
#include <frc2/command/commands.h>
#include <frc2/command/button/CommandXboxController.h>

namespace cmd {
frc2::CommandPtr ArmToAmpPos();
frc2::CommandPtr ArmToTrapPos();
frc2::CommandPtr ArmToStow();
frc2::CommandPtr IntakefullSequence();
frc2::CommandPtr AutoShootFullSequence();
frc2::CommandPtr StartTrapSequence();
frc2::CommandPtr EndTrapSequence();
frc2::CommandPtr OuttakeNote();
frc2::CommandPtr ShootFullSequenceWithVision(frc2::CommandXboxController& controller);
frc2::CommandPtr ShootFullSequenceWithoutVision();
frc2::CommandPtr ShootSpeakerOrArm();
frc2::CommandPtr FeedNoteToShooter();
frc2::CommandPtr PrepareToShoot();
frc2::CommandPtr ShootIntoAmp();
frc2::CommandPtr PassNote();
frc2::CommandPtr OuttakeIntakeAndEndEffector();
frc2::CommandPtr ShootAmpOrTrap();
frc2::CommandPtr IntakeFromSource();
}  // namespace cmd
