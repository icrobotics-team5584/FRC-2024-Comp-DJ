#include "commands/VisionCommands.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubVision.h"

namespace cmd {
using namespace frc2::cmd;

frc2::CommandPtr VisionAlignToSpeaker(frc2::CommandXboxController& controller) {
  static units::degree_t camYaw = 0_deg;
  static units::degree_t startingGyroYaw = 0_deg;

  return RunOnce([] {camYaw = 0_deg;
    startingGyroYaw = SubDrivebase::GetInstance().GetHeading().Degrees(); })
      .AndThen(Run([] {
        auto result = SubVision::GetInstance().GetSpecificTagYaw(SubVision::SPEAKER);

        if (result.has_value()) {
          camYaw = SubVision::GetInstance().GetSpecificTagYaw(SubVision::SPEAKER).value_or(0_deg);
          startingGyroYaw = SubDrivebase::GetInstance().GetHeading().Degrees();
        }


        units::degree_t currentGyroYaw = SubDrivebase::GetInstance().GetHeading().Degrees();
        units::degree_t gyroAngleTravelled = currentGyroYaw - startingGyroYaw;
        units::degree_t errorAngle = camYaw - gyroAngleTravelled;
        frc::SmartDashboard::PutNumber("Vision/Result", result.value_or(0_deg).value());
        frc::SmartDashboard::PutNumber("Vision/currentGyroYaw ", currentGyroYaw.value());
        frc::SmartDashboard::PutNumber("Vision/startingGyroYaw ", startingGyroYaw.value());
        frc::SmartDashboard::PutNumber("Vision/GyroAngleTravelled ", gyroAngleTravelled.value());
        frc::SmartDashboard::PutNumber("Vision/ErrorAngle ", errorAngle.value());
        
        SubDrivebase::GetInstance().RotateToZero(errorAngle);
        
      }))
      .AlongWith(SubDrivebase::GetInstance().JoystickDrive(controller, true))
      .FinallyDo([]{SubDrivebase::GetInstance().StopDriving();});
}

frc2::CommandPtr ShootSequence(frc2::CommandXboxController& controller) {
  return VisionAlignToSpeaker(controller).Until([] {
    return SubVision::GetInstance().IsOnTarget(SubVision::SPEAKER);
  });
}





frc2::CommandPtr VisionAlignToAmp(frc2::CommandXboxController& controller) {
  static units::degree_t camYaw = 0_deg;
  static units::degree_t startingGyroYaw = 0_deg;

  return Run([] {
           auto result = SubVision::GetInstance().GetSpecificTagYaw(SubVision::AMP);

           if (result.has_value()) {
             camYaw =
                 SubVision::GetInstance().GetSpecificTagYaw(SubVision::AMP).value_or(0_deg);
             startingGyroYaw = SubDrivebase::GetInstance().GetHeading().Degrees();
           }

           units::degree_t currentGyroYaw = SubDrivebase::GetInstance().GetHeading().Degrees();
           units::degree_t gyroAngleTravelled = currentGyroYaw - startingGyroYaw;
           units::degree_t errorAngle = camYaw - gyroAngleTravelled;

           SubDrivebase::GetInstance().RotateToZero(errorAngle);
         })
      .AlongWith(SubDrivebase::GetInstance().JoystickDrive(controller, true));
}

frc2::CommandPtr VisionRotateToTrap() {
  return Run(
             [] {
               frc::SmartDashboard::PutBoolean("Vision/Running vision rotate to Trap ", true);
               auto trapAngle = SubVision::GetInstance().getTrapAngle();

               frc::SmartDashboard::PutNumber("Vision/Angle of Trap ", trapAngle.value());
               auto currentGyroYaw = SubDrivebase::GetInstance().GetHeading().Degrees();

               frc::SmartDashboard::PutNumber("Vision/Current yaw ", currentGyroYaw.value());
               auto errorYaw = trapAngle - currentGyroYaw;
               SubDrivebase::GetInstance().RotateToZero(-1 * errorYaw);
             },
             {&SubDrivebase::GetInstance()})
      .Until([] {
        auto trapAngle = SubVision::GetInstance().getTrapAngle();
        auto currentGyroYaw = SubDrivebase::GetInstance().GetHeading().Degrees();
        auto errorYaw = units::math::fmod(trapAngle - currentGyroYaw, 360_deg);
        frc::SmartDashboard::PutNumber("Vision/Trap Error Yaw ", errorYaw.value());
        auto boolOnTarget = units::math::abs(errorYaw) < 2_deg;
        frc::SmartDashboard::PutBoolean("Vision/Is Trap rotate on target ", boolOnTarget);
        return boolOnTarget;
      });
}

frc2::CommandPtr VisionTranslateToTrap() {
  return Run(
             [] {
               frc::SmartDashboard::PutBoolean("Vision/Running vision translate to trap ", true);
               auto result = SubVision::GetInstance().getCamToTrapYaw();
               frc::SmartDashboard::PutNumber("Vision/result ", result.value_or(1000_deg).value());
               SubDrivebase::GetInstance().RotateToZero(0_deg);
               SubDrivebase::GetInstance().TranslateToZero(-result.value_or(0_deg));
             })
      .FinallyDo([] {
        frc::SmartDashboard::PutBoolean("Vision/Running vision translate to trap ", false);
      });
}

frc2::CommandPtr VisionAlignToClimb() {
  return VisionRotateToTrap().AlongWith(VisionTranslateToTrap());
}



}  // namespace cmd
