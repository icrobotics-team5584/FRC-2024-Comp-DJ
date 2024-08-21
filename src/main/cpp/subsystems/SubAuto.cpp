// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubAuto.h"
#include <choreo/lib/Choreo.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <frc2/command/CommandPtr.h>
#include "subsystems/SubArm.h"

using namespace frc2;

SubAuto::SubAuto() = default;

// This method will be called once per scheduler run
void SubAuto::Periodic() {}


//Description of total auton
frc2::CommandPtr SubAuto::CloseNotesAuto(){
    //Go from start pos to note 1
    return RunOnce([this]{pathplanner::PathPlannerAuto("Center to 1").ToPtr();})
    .AndThen(
        //Either: go from note 1 to subwoofer or go from note 1 to note 2
       [this]{return cmd::Either(
            //Go from note 1 to subwoofer and shoot
            pathplanner::PathPlannerAuto("1 to Shoot").ToPtr().AndThen([this]{
                //Then go from subwoofer to note 2
                pathplanner::PathPlannerAuto("Shoot to 2").ToPtr()
                    ;})
            , 
            pathplanner::PathPlannerAuto("1 to 2").ToPtr().AndThen(
                                cmd::Either(
                    pathplanner::PathPlannerAuto("Path if decision yes").ToPtr()
                    ,
                    pathplanner::PathPlannerAuto("Path if decision no").ToPtr()
                    ,
                    []{return true;} //Selector bool
                )
            )
            ,
     [] {return SubArm::GetInstance().CheckIfArmHasGamePiece();});})}

/* TEMPLATE
    return RunOnce([this]{pathplanner::PathPlannerAuto("firstPath").ToPtr();})
    .AndThen(
        cmd::Either(
            pathplanner::PathPlannerAuto("Path if decision yes").ToPtr().AndThen(
                cmd::Either(
                    pathplanner::PathPlannerAuto("Path if decision yes").ToPtr()
                    ,
                    pathplanner::PathPlannerAuto("Path if decision no").ToPtr()
                    ,
                    []{return true;} //Selector bool
                )
            )  
            , 
            pathplanner::PathPlannerAuto("Path if decision no").ToPtr().AndThen(
                                cmd::Either(
                    pathplanner::PathPlannerAuto("Path if decision yes").ToPtr()
                    ,
                    pathplanner::PathPlannerAuto("Path if decision no").ToPtr()
                    ,
                    []{return true;} //Selector bool
                )
            )
            ,
     [] {return true Selector bool;}));
     */
