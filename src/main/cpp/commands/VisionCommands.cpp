#include "commands/VisionCommands.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubVision.h"

namespace cmd{
    using namespace frc2::cmd;

    frc2::CommandPtr VisionRotateToZero(){
        return Run(
            []{SubDrivebase::GetInstance().RotateToZero(SubVision::GetInstance().GetSpecificTagYaw(SubVision::SPEAKER).value());},
            {&SubDrivebase::GetInstance()}
        );}
        
    frc2::CommandPtr ShootSequence(){
        return VisionRotateToZero().Until([]{return SubVision::GetInstance().IsOnTarget(SubVision::SPEAKER);});
    }

} // namespace cmd
