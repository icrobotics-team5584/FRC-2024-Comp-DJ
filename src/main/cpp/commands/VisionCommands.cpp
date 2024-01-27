#include "commands/VisionCommands.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubVision.h"

namespace cmd{
    using namespace frc2::cmd;

    frc2::CommandPtr VisionRotateToZero(){
        return Run(
            []{SubDrivebase::GetInstance().RotateToZero(-SubVision::GetInstance().BestTargetYaw());},
            {&SubDrivebase::GetInstance()}
        );}
        
    frc2::CommandPtr ShootSequence(){
        return VisionRotateToZero().Until([]{return SubVision::GetInstance().IsOnTarget();});
    }

} // namespace cmd
