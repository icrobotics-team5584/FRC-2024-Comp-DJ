// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubIntake.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc2::cmd;

SubIntake::SubIntake() {
  frc::SmartDashboard::PutString("Intake/Intake Deploy State: ", "Intake retracted");
  _intakeMotorSpin.RestoreFactoryDefaults();
  _intakeMotorSpin.SetSmartCurrentLimit(60);
  _intakeMotorSpin.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus0, 500);
  _intakeMotorSpin.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus1, 500);
  _intakeMotorSpin.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus2, 500);
  _intakeMotorSpin.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus3, 500);
  _intakeMotorSpin.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus4, 500);
  _intakeMotorSpin.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus5, 500);
  _intakeMotorSpin.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus6, 500);
}
// This method will be called once per scheduler run
void SubIntake::Periodic() {
  frc::SmartDashboard::PutNumber("Intake/Intake piston state", solIntake.Get());
  frc::SmartDashboard::PutNumber("Intake/Intake Motor: ", _intakeMotorSpin.Get());
  frc::SmartDashboard::PutBoolean("Intake/Extended Reed Switch", _intakeExtendedReed.Get());
  frc::SmartDashboard::PutNumber("Intake/speed", encoder.GetVelocity());
}

void SubIntake::SimulationPeriodic(){
  _simIntakeExtendedReed.SetValue(solIntake.Get() == frc::DoubleSolenoid::Value::kForward);
}

//Extend intake
frc2::CommandPtr SubIntake::ExtendIntake() {
  return RunOnce([this] { solIntake.Set(frc::DoubleSolenoid::kForward); });
}

//Extend intake via toggle switch
frc2::CommandPtr SubIntake::ToggleExtendIntake() {
  return StartEnd([this] { solIntake.Set(frc::DoubleSolenoid::kForward); },
                  [this] { solIntake.Set(frc::DoubleSolenoid::kReverse); });
}

//Stop spinning intake by setting intake motor to 0% speed
frc2::CommandPtr SubIntake::StopSpinningIntake() {
  return RunOnce([this] { _intakeMotorSpin.Set(0); });
}

//Set intake motor to 100% speed, after cancelled turn off intake motor
frc2::CommandPtr SubIntake::StartSpinningIntake() {
  return Run([this] { _intakeMotorSpin.Set(1); }).FinallyDo([this]{_intakeMotorSpin.Set(0);});
}

//Set intake motor to -100% speed, after cancelled turn off intake motor
frc2::CommandPtr SubIntake::Outtake() {
  return Run([this]{ _intakeMotorSpin.Set(-1);}).FinallyDo([this]{_intakeMotorSpin.Set(0);});
}

//used with IntakeSequence(), extends intake then waits 0.1 seconds then turns intake motor to 100% power
frc2::CommandPtr SubIntake::Intake(){
  return ExtendIntake().AndThen(Wait(0.1_s)).AndThen(StartSpinningIntake());
}

//Set intake power to 0% power
frc2::CommandPtr SubIntake::EndIntake(){
  return StopSpinningIntake();
}

//runs the Intake command, then when cancelled set intake motor to 0% power and retract
frc2::CommandPtr SubIntake::IntakeSequence(){
  return Intake()
      .FinallyDo([this] {
        solIntake.Set(frc::DoubleSolenoid::Value::kReverse);
        _intakeMotorSpin.Set(0);
      });
}

//Check if intake is deployed or not via reed switch
bool SubIntake::IsIntakeDeployed(){
  return !_intakeExtendedReed.Get();
}  // LOCK ARM IF RETURN FALSE

//The function version of retracting the intake
void SubIntake::FuncRetractIntake(){
  solIntake.Set(frc::DoubleSolenoid::kReverse);
}

//The command version of retracting the intake
 frc2::CommandPtr SubIntake::CommandRetractIntake(){
  return RunOnce([this]{solIntake.Set(frc::DoubleSolenoid::kReverse);});
 }