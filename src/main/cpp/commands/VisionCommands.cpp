#include "commands/VisionCommands.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubVision.h"

namespace cmd{
    using namespace frc2::cmd;

/*
    frc2::CommandPtr VisionRotateToZero(){
        return Run(
            []{SubDrivebase::GetInstance().RotateToZero(SubVision::GetInstance().GetSpecificTagYaw(SubVision::SPEAKER).value_or(0_deg));},
            {&SubDrivebase::GetInstance()}
        );
    }
*/

    frc2::CommandPtr VisionRotateToZero(){

        static units::degree_t camYaw = 0_deg;
        static units::degree_t startingGyroYaw = 0_deg;

        return RunOnce(
            []{
                camYaw = SubVision::GetInstance().GetSpecificTagYaw(SubVision::SPEAKER).value_or(0_deg);
                startingGyroYaw = SubDrivebase::GetInstance().GetHeading().Degrees();
            }
        ).AndThen(
            Run(
                []{
                    units::degree_t currentGyroYaw = SubDrivebase::GetInstance().GetHeading().Degrees();
                    units::degree_t gyroAngleTravelled = currentGyroYaw - startingGyroYaw;
                    units::degree_t errorAngle = camYaw - gyroAngleTravelled;

                    SubDrivebase::GetInstance().RotateToZero(errorAngle);
            }));}
    
        
    frc2::CommandPtr ShootSequence(){
        return VisionRotateToZero().Until([]{return SubVision::GetInstance().IsOnTarget(SubVision::SPEAKER);});
    }

} // namespace cmd
