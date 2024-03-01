#include "commands/VisionCommands.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubVision.h"

namespace cmd {
using namespace frc2::cmd;

frc2::CommandPtr VisionRotateToZero(frc2::CommandXboxController& controller) {
  static units::degree_t camYaw = 0_deg;
  static units::degree_t startingGyroYaw = 0_deg;

  return (Run(
      []{ auto result = SubVision::GetInstance().GetSpecificTagYaw(SubVision::SPEAKER);

        if (result.has_value()) {
          camYaw = SubVision::GetInstance().GetSpecificTagYaw(SubVision::SPEAKER).value_or(0_deg);
          startingGyroYaw = SubDrivebase::GetInstance().GetHeading().Degrees();
        }

        units::degree_t currentGyroYaw = SubDrivebase::GetInstance().GetHeading().Degrees();
        units::degree_t gyroAngleTravelled = currentGyroYaw - startingGyroYaw;
        units::degree_t errorAngle = camYaw - gyroAngleTravelled;

        SubDrivebase::GetInstance().RotateToZero(errorAngle);
      }).AlongWith(SubDrivebase::GetInstance().JoystickDrive(controller, true)));
}

frc2::CommandPtr ShootSequence(frc2::CommandXboxController& controller) {
  return VisionRotateToZero(controller).Until(
      [] { return SubVision::GetInstance().IsOnTarget(SubVision::SPEAKER); });
}

}  // namespace cmd
