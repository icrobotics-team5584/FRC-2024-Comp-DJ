// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/commands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

class SubAuto : public frc2::SubsystemBase {
 public:
  static SubAuto& GetInstance() {
    static SubAuto inst;
    return inst;
  }
  SubAuto();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  frc2::CommandPtr CloseNotesAuto();
  frc2::CommandPtr SimpleAuto();
  frc2::CommandPtr CNA1();
  frc2::CommandPtr CNA2();
  frc2::CommandPtr CNA3();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  frc2::CommandPtr _centerTo1 = pathplanner::PathPlannerAuto("Center to 1").ToPtr();
  frc2::CommandPtr _1ToShoot = pathplanner::PathPlannerAuto("1 to Shoot").ToPtr();
  frc2::CommandPtr _1To2 = pathplanner::PathPlannerAuto("1 to 2").ToPtr();
  frc2::CommandPtr _shootTo2 = pathplanner::PathPlannerAuto("Shoot to 2").ToPtr();
  frc2::CommandPtr _2ToShoot = pathplanner::PathPlannerAuto("2 to Shoot").ToPtr();
  frc2::CommandPtr _2To3 = pathplanner::PathPlannerAuto("2 to 3").ToPtr();
  frc2::CommandPtr _shootTo3 = pathplanner::PathPlannerAuto("Shoot to 3").ToPtr();
  frc2::CommandPtr _3ToShoot = pathplanner::PathPlannerAuto("3 to Shoot").ToPtr();
  frc2::CommandPtr _3To4 = pathplanner::PathPlannerAuto("3 to 4").ToPtr();
  frc2::CommandPtr _shootTo4 = pathplanner::PathPlannerAuto("Shoot to 4").ToPtr();
  frc2::CommandPtr _4ToShoot = pathplanner::PathPlannerAuto("4 to Shoot").ToPtr();
  frc2::CommandPtr _4To5 = pathplanner::PathPlannerAuto("4 to 5").ToPtr();
  frc2::CommandPtr _shootTo5 = pathplanner::PathPlannerAuto("Shoot to 5").ToPtr();
};
