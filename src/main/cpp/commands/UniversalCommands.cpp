#include "commands/UniversalCommands.h"
#include "subsystems/SubAmp.h"
#include "subsystems/SubClimber.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubIntake.h"
#include "subsystems/SubShooter.h"

namespace cmd {
using namespace frc2::cmd;

frc2::CommandPtr ArmToAmpPos() {
  return RunOnce([]() { SubIntake::GetInstance().ExtendIntake(); }, {&SubAmp::GetInstance()})
      .AndThen([]() { return SubAmp::GetInstance().CheckIfArmIsHome(); }, {&SubAmp::GetInstance()})
      .AndThen([]() { return SubAmp::GetInstance().TiltArmToAngle(SubAmp::AMP_ANGLE); });
}

frc2::CommandPtr ArmToTrapPos() {
  return RunOnce([]() { SubIntake::GetInstance().ExtendIntake(); }, {&SubAmp::GetInstance()})
      .AndThen([]() { return SubAmp::GetInstance().CheckIfArmIsHome(); }, {&SubAmp::GetInstance()})
      .AndThen([]() { return SubAmp::GetInstance().TiltArmToAngle(SubAmp::TRAP_ANGLE); });
}

frc2::CommandPtr ArmToStow() {
  return RunOnce([]() { SubIntake::GetInstance().RetractIntake(); }, {&SubAmp::GetInstance()})
      .AndThen([]() { return SubAmp::GetInstance().TiltArmToAngle(SubAmp::HOME_ANGLE); },
               {&SubAmp::GetInstance()})
      .AndThen([]() { return SubAmp::GetInstance().CheckIfArmIsHome(); });
}

frc2::CommandPtr SequenceArmToAmpPos() {
  return StartEnd([] { ArmToAmpPos(); }, [] { ArmToStow(); });
}

frc2::CommandPtr SequenceArmToTrapPos() {
  return StartEnd([] { ArmToTrapPos(); }, [] { ArmToStow(); });
}

frc2::CommandPtr ShootSequence() {
  return Run([] { /*AUTO VISION AIM COMMAND*/ })
      .Until(/*AUTO VISION CHECKER = true*/ {})
      .AndThen({SubShooter::GetInstance().ShootNote()});
}

frc2::CommandPtr IntakeSequence() {
  return StartEnd(
      [] {
        SubIntake::GetInstance().ExtendIntake().AndThen(
            {SubIntake::GetInstance().StartSpinningIntake()});
      },
      [] {
        SubIntake::GetInstance().StopSpinningIntake().AndThen(
            [] { SubIntake::GetInstance().RetractIntake(); });
      });
}
}  // namespace cmd
