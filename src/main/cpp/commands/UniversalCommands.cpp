#include "commands/UniversalCommands.h"
#include "subsystems/SubClimber.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubIntake.h"
#include "subsystems/SubShooter.h"
#include "subsystems/SubArm.h"
#include "commands/VisionCommands.h"
#include "subsystems/SubVision.h"
#include "RobotContainer.h"

namespace cmd {
using namespace frc2::cmd;

frc2::CommandPtr ArmToAmpPos() {
  return SubShooter::GetInstance()
      .ShooterChangePosClose()
      .AndThen(SubIntake::GetInstance().ExtendIntake())
      .AndThen(WaitUntil([] { return SubIntake::GetInstance().IsIntakeDeployed(); }))
      .WithTimeout(1_s)
      .AndThen(SubArm::GetInstance().TiltArmToAngle(SubArm::AMP_ANGLE));
}

frc2::CommandPtr ArmToTrapPos() {
return SubIntake::GetInstance()
      .ExtendIntake()
      .AndThen(WaitUntil([] { return SubIntake::GetInstance().IsIntakeDeployed(); })).WithTimeout(1_s)
      .AndThen(SubArm::GetInstance().TiltArmToAngle(SubArm::TRAP_ANGLE));
}

frc2::CommandPtr ArmToStow() {
  return SubArm::GetInstance()
      .TiltArmToAngle(SubArm::HOME_ANGLE)
      .AndThen([] { SubIntake::GetInstance().FuncRetractIntake(); });
}

frc2::CommandPtr ShootFullSequenceWithVision(frc2::CommandXboxController& controller) {
  return VisionAlignToSpeaker(controller).Until([]{return SubVision::GetInstance().IsOnTarget(SubVision::SPEAKER);})
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

frc2::CommandPtr ShootFullSequenceWithoutVision(){
  return SubShooter::GetInstance()
      .ShootSequence()
      .AlongWith(WaitUntil([] {
                   return SubShooter::GetInstance().CheckShooterSpeed();
                 }).AndThen(SubArm::GetInstance().FeedNote()))
      .AndThen(WaitUntil([] { return !SubShooter::GetInstance().CheckShooterLineBreak(); }))
      .AndThen(SubShooter::GetInstance().ShooterChangePosClose());
}

frc2::CommandPtr ShootSpeakerOrArm() {
  return Either(ShootFullSequenceWithoutVision(), ShootAmpOrTrap(),
                [] { return SubArm::GetInstance().GetAngle() < 0.4_tr; });
}

frc2::CommandPtr ShootAmpOrTrap(){
    return Either(SubArm::GetInstance().TrapShooter(), SubArm::GetInstance().AmpShooter(),
                  [] {
                    return SubClimber::GetInstance().CheckLeftClimberPos() &&
                           SubClimber::GetInstance().CheckRightClimberPos() < 0.1_m;
                  });
}

frc2::CommandPtr IntakefullSequence() {
  return SubIntake::GetInstance()
      .Intake()
      .AlongWith(SubArm::GetInstance().StoreNote())
      .Until([] { return SubArm::GetInstance().CheckIfArmHasGamePiece(); })
      .AndThen([] { SubIntake::GetInstance().FuncRetractIntake(); })
      .AndThen(SubIntake::GetInstance().Outtake().WithTimeout(0.3_s))
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
      .AndThen(SubIntake::GetInstance().Outtake())
      .AndThen(Idle())
      .FinallyDo([] { SubIntake::GetInstance().FuncRetractIntake(); });
}

frc2::CommandPtr OuttakeIntakeAndEndEffector() {
    return SubIntake::GetInstance()
      .ExtendIntake()
      .AndThen(SubIntake::GetInstance().Outtake())
      .AlongWith(SubArm::GetInstance().Outtake())
      .AlongWith(SubShooter::GetInstance().Outtake())
      .AndThen(Idle())
      .FinallyDo([] { SubIntake::GetInstance().FuncRetractIntake();});
}

frc2::CommandPtr FeedNoteToShooter() {
  return SubShooter::GetInstance()
      .StartFeeder()
      .AlongWith(SubArm::GetInstance().FeedNote())
      .Until([] { return SubShooter::GetInstance().CheckShooterLineBreak(); })
      .AndThen(SubShooter::GetInstance().ReverseFeeder().WithTimeout(0.2_s))
      .FinallyDo([] { SubShooter::GetInstance().StopFeederFunc(); });
}

frc2::CommandPtr PrepareToShoot() {
  return FeedNoteToShooter().AndThen(SubShooter::GetInstance().StartShooter());
}

frc2::CommandPtr ShootIntoAmp() {
  return SubShooter::GetInstance()
      .ShootIntoAmpSequence()
      .AlongWith(WaitUntil([] {
                   return SubShooter::GetInstance().CheckShooterSpeed();
                 }).AndThen(SubArm::GetInstance().FeedNote()))
      .AndThen(WaitUntil([] { return !SubShooter::GetInstance().CheckShooterLineBreak(); }))
      .AndThen(SubShooter::GetInstance().ShooterChangePosClose());
      
}

frc2::CommandPtr PassNote() {
    return SubShooter::GetInstance()
      .ShooterPassNoteSequence()
      .AlongWith(WaitUntil([] {
                   return SubShooter::GetInstance().CheckShooterSpeed();
                 }).AndThen(SubArm::GetInstance().FeedNote()))
      .AndThen(WaitUntil([] { return !SubShooter::GetInstance().CheckShooterLineBreak(); }))
      .AndThen(SubShooter::GetInstance().ShooterChangePosClose());
}

frc2::CommandPtr IntakeFromSource() {
  return SubShooter::GetInstance()
      .IntakeFromSource()
      .AlongWith(SubArm::GetInstance().Outtake())
      .Until([] { return SubArm::GetInstance().CheckIfArmHasGamePiece(); })
      .FinallyDo([] {
        SubShooter::GetInstance().StopShooterFunc();
        SubArm::GetInstance().StopEndEffector();
      });
}

}  // namespace cmd