#include "commands/UniversalCommands.h"
#include "subsystems/SubClimber.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubIntake.h"
#include "subsystems/SubShooter.h"
#include "subsystems/SubArm.h"

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
      .Until([] { return SubArm::GetInstance().CheckIfArmIsHome(); })
      .AndThen([] { SubIntake::GetInstance().FuncRetractIntake(); });
}

frc2::CommandPtr SequenceArmToAmpPos() {
  return StartEnd([] { ArmToAmpPos(); }, [] { ArmToStow(); });
}

frc2::CommandPtr SequenceArmToTrapPos() {
  return StartEnd([] { ArmToTrapPos(); }, [] { ArmToStow(); });
}

frc2::CommandPtr ShootFullSequence() {
  return Run([] { /*AUTO VISION AIM COMMAND*/ })
      .Until([] { return true; })
      .AndThen({SubShooter::GetInstance().ShootSequence()})
      .AlongWith(WaitUntil([] {
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


frc2::CommandPtr TrapSequence() {
  return ArmToTrapPos()
        .AndThen(SubAmp::GetInstance().AmpShooter().WithTimeout(1_s));
}

}  // namespace cmd