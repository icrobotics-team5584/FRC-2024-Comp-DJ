#include "commands/VisionCommands.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubVision.h"

namespace cmd{
    using namespace frc2::cmd;

    frc2::CommandPtr VisionRotateToZero(){
        return Run(
            []{SubDrivebase::GetInstance().RotateToZero(SubVision::GetInstance().GetSpecificTagYaw(SubVision::SPEAKER).value_or(0_deg));},
            {&SubDrivebase::GetInstance()}
        );
    }
    
    /*
    frc2::CommandPtr VisionClimb(){
        // find yaw
        // start climb sequence
        // balance
        // trap
    }
    */
        
    frc2::CommandPtr ShootSequence(){
        return VisionRotateToZero().Until([]{return SubVision::GetInstance().IsOnTarget(SubVision::SPEAKER);});
    }

} // namespace cmd
