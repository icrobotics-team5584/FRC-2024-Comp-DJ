#include <commands/ClimberCommands.h>
#include <subsystems/SubClimber.h>

namespace cmd {
    using namespace frc2::cmd;

    frc2::CommandPtr ClimberUp() {
        return RunOnce([] {SubClimber::GetInstance().Extend();});
    }

    frc2::CommandPtr ClimberDown() {
        return RunOnce([] {SubClimber::GetInstance().Retract();});
    }
}