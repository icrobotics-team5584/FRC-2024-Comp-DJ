#include <commands/ClimberCommands.h>
#include <subsystems/SubClimber.h>

namespace cmd {
    using namespace frc2::cmd;

    frc2::CommandPtr ClimberExtend() {
        return RunOnce([] {SubClimber::GetInstance().Extend();});
    }

    frc2::CommandPtr ClimberRetract() {
        return RunOnce([] {SubClimber::GetInstance().Retract();});
    }

    frc2::CommandPtr ClimberExtendManual() {
        return RunOnce([] {SubClimber::GetInstance().Start(0.5);});
    }

    frc2::CommandPtr ClimberRetractManual() {
        return RunOnce([] {SubClimber::GetInstance().Start(-0.5);});
    }

    frc2::CommandPtr ClimberStop() {
        return RunOnce([] {SubClimber::GetInstance().Stop();});
    }
}