#include <frc2/command/commands.h>

namespace cmd {
    frc2::CommandPtr ClimberExtend();
    frc2::CommandPtr ClimberRetract();
    frc2::CommandPtr ClimberExtendManual();
    frc2::CommandPtr ClimberRetractManual();
    frc2::CommandPtr ClimberStop();
}