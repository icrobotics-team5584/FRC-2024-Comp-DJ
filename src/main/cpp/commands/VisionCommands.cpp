#include "commands/VisionCommands.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubVision.h"

namespace cmd {
using namespace frc2::cmd;

frc2::CommandPtr VisionRotateToSpeaker(frc2::CommandXboxController& controller) {
  static units::degree_t camYaw = 0_deg;
  static units::degree_t startingGyroYaw = 0_deg;

  return Run([] {
           auto result = SubVision::GetInstance().GetSpecificTagYaw(SubVision::SPEAKER);

           if (result.has_value()) {
             camYaw =
                 SubVision::GetInstance().GetSpecificTagYaw(SubVision::SPEAKER).value_or(0_deg);
             startingGyroYaw = SubDrivebase::GetInstance().GetHeading().Degrees();
           }

           units::degree_t currentGyroYaw = SubDrivebase::GetInstance().GetHeading().Degrees();
           units::degree_t gyroAngleTravelled = currentGyroYaw - startingGyroYaw;
           units::degree_t errorAngle = camYaw - gyroAngleTravelled;

           SubDrivebase::GetInstance().RotateToZero(errorAngle);
         })
      .AlongWith(SubDrivebase::GetInstance().JoystickDrive(controller, true));
}

frc2::CommandPtr VisionTranslateToTrap(frc2::CommandXboxController& controller) {
  static units::degree_t camYaw = 0_deg;
  static units::degree_t startingGyroYaw = 0_deg;

  return Run([] {
           auto result = SubVision::GetInstance().GetSpecificTagYaw(SubVision::SPEAKER);

           if (result.has_value()) {
             camYaw =
                 SubVision::GetInstance().GetSpecificTagYaw(SubVision::SPEAKER).value_or(0_deg);
             startingGyroYaw = SubDrivebase::GetInstance().GetHeading().Degrees();
           }

           units::degree_t currentGyroYaw = SubDrivebase::GetInstance().GetHeading().Degrees();
           units::degree_t gyroAngleTravelled = currentGyroYaw - startingGyroYaw;
           units::degree_t errorAngle = camYaw - gyroAngleTravelled;

           SubDrivebase::GetInstance().RotateToZero(errorAngle);
         })
      .AlongWith(SubDrivebase::GetInstance().JoystickDrive(controller, true));
}

frc2::CommandPtr ShootSequence(frc2::CommandXboxController& controller) {
  return VisionRotateToSpeaker(controller).Until([] {
    return SubVision::GetInstance().IsOnTarget(SubVision::SPEAKER);
  });
}

frc2::CommandPtr VisionRotateToTrap() {
  return Run([] {
    auto trapAngle = SubVision::GetInstance().getTrapAngle();
    frc::SmartDashboard::PutNumber("Vision/Angle of Trap: ", trapAngle.value());
    auto currentGyroYaw = SubDrivebase::GetInstance().GetHeading().Degrees();
    frc::SmartDashboard::PutNumber("Vision/Current yaw: ", currentGyroYaw.value());
    auto errorYaw = trapAngle - currentGyroYaw;
    frc::SmartDashboard::PutNumber("Vision/Error Yaw: ", errorYaw.value());
    SubDrivebase::GetInstance().RotateToZero(errorYaw);
  },{&SubDrivebase::GetInstance()}).Until([]{
    return false;
    /*
    auto trapAngle = SubVision::GetInstance().getTrapAngle();
    auto currentGyroYaw = SubDrivebase::GetInstance().GetHeading().Degrees();
    auto errorYaw = trapAngle - currentGyroYaw;
    return units::math::abs(errorYaw) < 4_deg;
    */
  });
}

frc2::CommandPtr VisionTranslateToTrap() {

  static units::degree_t camYaw = 0_deg;
  static units::degree_t startingGyroYaw = 0_deg;

  return Run([] {
    auto result = SubVision::GetInstance().GetSpecificTagYaw(SubVision::SPEAKER);

    if (result.has_value()) {
      camYaw = SubVision::GetInstance().GetSpecificTagYaw(SubVision::SPEAKER).value_or(0_deg);
      startingGyroYaw = SubDrivebase::GetInstance().GetHeading().Degrees();
    }

    SubDrivebase::GetInstance().TranslateToZero(camYaw);
  });
}

frc2::CommandPtr VisionClimb(){
    return VisionRotateToTrap().AndThen(VisionTranslateToTrap());
}

}  // namespace cmd
