#include "commands/UniversalCommands.h"
#include "subsystems/SubAmp.h"
#include "subsystems/SubClimber.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubIntake.h"
#include "subsystems/SubShooter.h"

namespace cmd {
using namespace frc2::cmd;

frc2::CommandPtr ArmToAmpPos() {
  return SubIntake::GetInstance().ExtendIntake()
      .AndThen(SubAmp::GetInstance().TiltArmToAngle(SubAmp::AMP_ANGLE));
}

frc2::CommandPtr ArmToTrapPos() {
  return RunOnce([]() { SubIntake::GetInstance().ExtendIntake(); }, {&SubAmp::GetInstance()})
      .AndThen([]() { return SubAmp::GetInstance().CheckIfArmIsHome(); }, {&SubAmp::GetInstance()})
      .AndThen([]() { return SubAmp::GetInstance().TiltArmToAngle(SubAmp::TRAP_ANGLE); });
}

frc2::CommandPtr ArmToStow() {
  return SubAmp::GetInstance()
      .TiltArmToAngle(SubAmp::HOME_ANGLE)
      .Until([] { return SubAmp::GetInstance().CheckIfArmIsHome(); })
      .AndThen([] { SubIntake::GetInstance().RetractIntake(); });
}

frc2::CommandPtr SequenceArmToAmpPos() {
  return StartEnd([] { ArmToAmpPos(); }, [] { ArmToStow(); });
}

frc2::CommandPtr SequenceArmToTrapPos() {
  return StartEnd([] { ArmToTrapPos(); }, [] { ArmToStow(); });
}

frc2::CommandPtr ShootFullSequence() {
  return Run([] { /*AUTO VISION AIM COMMAND*/ })
      .Until([] {return true;})
      .AndThen({SubShooter::GetInstance().ShootSequence()});
}

frc2::CommandPtr IntakefullSequence(){
  return SubIntake::GetInstance()
      .Intake()
      .AlongWith(SubAmp::GetInstance().StoreNote())
      .FinallyDo([] { SubIntake::GetInstance().RetractIntake(); });
}
}  // namespace cmd
