#include "commands/UniversalCommands.h"
#include "subsystems/SubClimber.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubIntake.h"
#include "subsystems/SubShooter.h"
#include "subsystems/SubArm.h"
#include "commands/VisionCommands.h"
#include "subsystems/SubVision.h"

namespace cmd {
using namespace frc2::cmd;

frc2::CommandPtr ArmToAmpPos() {
  return SubIntake::GetInstance()
      .ExtendIntake()
      .AndThen(SubArm::GetInstance().TiltArmToAngle(SubArm::AMP_ANGLE))
      .AndThen(SubArm::GetInstance().FastAmpShooter().WithTimeout(3_s));
}

frc2::CommandPtr ArmToTrapPos() {
  return RunOnce([]() { SubIntake::GetInstance().ExtendIntake(); }, {&SubArm::GetInstance()})
      .AndThen([]() { return SubArm::GetInstance().CheckIfArmIsHome(); }, {&SubArm::GetInstance()})
      .AndThen([]() { return SubArm::GetInstance().TiltArmToAngle(SubArm::TRAP_ANGLE); });
}

frc2::CommandPtr ArmToStow() {
  return SubArm::GetInstance()
      .TiltArmToAngle(SubArm::HOME_ANGLE)
      .Until([] { return SubArm::GetInstance().CheckIfArmIsHome(); });/*
      .AndThen([] { SubIntake::GetInstance().FuncRetractIntake(); });*/
}

frc2::CommandPtr SequenceArmToAmpPos() {
  return StartEnd([] { ArmToAmpPos(); }, [] { ArmToStow(); });
}

frc2::CommandPtr SequenceArmToTrapPos() {
  return StartEnd([] { ArmToTrapPos(); }, [] { ArmToStow(); });
}

frc2::CommandPtr ShootFullSequenceWithVision() {
  return VisionRotateToZero().Until([]{return SubVision::GetInstance().IsOnTarget(SubVision::SPEAKER);})
      .Until([] { return true; })
      .AndThen({SubShooter::GetInstance().ShootSequence()})
      .AlongWith(WaitUntil([] {
                   return SubShooter::GetInstance().CheckShooterSpeed();
                 }).AndThen({SubArm::GetInstance().FeedNote()}));
}

frc2::CommandPtr AutoShootFullSequence() {
      return SubShooter::GetInstance().AutoShootSequence()
      .AndThen({SubArm::GetInstance().FeedNote()});
}

frc2::CommandPtr ShootFullSequenceWithoutVision() {
  return SubShooter::GetInstance().ShootSequence().AlongWith(
      WaitUntil([] {
        return SubShooter::GetInstance().CheckShooterSpeed();
      }).AndThen({SubArm::GetInstance().FeedNote()}));
}

frc2::CommandPtr IntakefullSequence(){
  return SubIntake::GetInstance()
      .Intake()
      .AlongWith(SubArm::GetInstance().StoreNote())
      .Until([]{return SubArm::GetInstance().CheckIfArmHasGamePiece();})
      .FinallyDo([] { SubIntake::GetInstance().FuncRetractIntake(); });
}


frc2::CommandPtr StartTrapSequence() {
  return SubIntake::GetInstance().ExtendIntake().AndThen(ArmToTrapPos()).AndThen(SubShooter::GetInstance().StartShooter());
}

frc2::CommandPtr EndTrapSequence() {
  return cmd::ArmToStow();
}

frc2::CommandPtr OuttakeNote() {
  return SubIntake::GetInstance()
      .ExtendIntake()
      //.AndThen(SubIntake::GetInstance().Outtake())
      //.AlongWith(SubArm::GetInstance().Outtake())
     // .AlongWith(SubShooter::GetInstance().Outtake())
      .AndThen(Idle())
      .FinallyDo([] { SubIntake::GetInstance().FuncRetractIntake(); });
}

frc2::CommandPtr FeedNoteToShooter() {
  return SubShooter::GetInstance()
      .StartFeederSlow()
      .AlongWith(SubArm::GetInstance().FeedNote())
      .Until([] { return SubShooter::GetInstance().CheckShooterLineBreak(); })
      .FinallyDo([] { SubShooter::GetInstance().StopFeederFunc(); });
}

frc2::CommandPtr PrepareToShoot() {
  return FeedNoteToShooter().AndThen(SubShooter::GetInstance().StartShooter());
}

}  // namespace cmd