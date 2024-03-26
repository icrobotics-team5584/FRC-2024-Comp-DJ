// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubClimber.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <cameraserver/CameraServer.h>

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <cameraserver/CameraServer.h>
#include <subsystems/SubShooter.h>
#include "utilities/SwerveModule.h"

void Robot::RobotInit() {
  frc::DataLogManager::Start();
  frc::CameraServer::StartAutomaticCapture();
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog(),true);
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
  //frc::SmartDashboard::PutNumber("Compressor PSI", m_container._compressor.GetPressure().value());
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
  SubDrivebase::GetInstance().SyncSensors();
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
  SubDrivebase::GetInstance().SyncSensors();
  SubShooter::GetInstance().StopShooterFunc();
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
