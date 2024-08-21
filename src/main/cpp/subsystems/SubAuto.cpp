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

<<<<<<< HEAD

// //Description of total auton
// frc2::CommandPtr SubAuto::CloseNotesAuto(){
//     //Go from start pos to note 1
//     return RunOnce([this]{pathplanner::PathPlannerAuto("Center to 1").ToPtr();})
//     .AndThen(
//         //Either: go from note 1 to subwoofer or go from note 1 to note 2
//        [this]{return cmd::Either(
//             //Go from note 1 to subwoofer and shoot
//             pathplanner::PathPlannerAuto("1 to Shoot").ToPtr().AndThen([this]{
//                 //Then go from subwoofer to note 2
//                 pathplanner::PathPlannerAuto("Shoot to 2").ToPtr()
//                     ;})
//             , 
//             pathplanner::PathPlannerAuto("1 to 2").ToPtr().AndThen(
//                                 cmd::Either(
//                     pathplanner::PathPlannerAuto("Path if decision yes").ToPtr()
//                     ,
//                     pathplanner::PathPlannerAuto("Path if decision no").ToPtr()
//                     ,
//                     []{return true;} //Selector bool
//                 )
//             )
//             ,
//      [] {return SubArm::GetInstance().CheckIfArmHasGamePiece();});})}
=======
// Description of total auton
frc2::CommandPtr SubAuto::CloseNotesAuto() {
  // Go from start pos to note 1
  return RunOnce([this] { pathplanner::PathPlannerAuto("Center to 1").ToPtr(); })
      .AndThen(
          // Either: go from note 1 to subwoofer or go from note 1 to note 2
          [this] {
            return cmd::Either(
                // Go from note 1 to subwoofer and shoot
                pathplanner::PathPlannerAuto("1 to Shoot")
                    .ToPtr()
                    .AndThen([this] {
                      // Then go from subwoofer to note 2
                      pathplanner::PathPlannerAuto("Shoot to 2").ToPtr();
                    })
                    .AndThen([this] {
                      // Either: go from note 2 to subwoofer or go from note 2 to note 3
                      return cmd::Either(
                          // Either:go from note 2 to subwoofer and shoot. END AUTO or go from 2 to
                          // 3
                          pathplanner::PathPlannerAuto("2 to Shoot").ToPtr(),
                          pathplanner::PathPlannerAuto("2 to 3").ToPtr().AndThen([this] {
                            return pathplanner::PathPlannerAuto("3 to Shoot").ToPtr().OnlyIf([] {
                              return SubArm::GetInstance().CheckIfArmHasGamePiece();
                            });
                          }),
                          [] { return SubArm::GetInstance().CheckIfArmHasGamePiece(); });
                    }),
                pathplanner::PathPlannerAuto("1 to 2").ToPtr().AndThen(cmd::Either(
                    pathplanner::PathPlannerAuto("2 to Shoot").ToPtr().AndThen(
                    pathplanner::PathPlannerAuto("Shoot to 3").ToPtr().OnlyIf([] {
                        return SubArm::GetInstance().CheckIfArmHasGamePiece();
                      })
                    ),
                    pathplanner::PathPlannerAuto("2 to 3").ToPtr().AndThen([this] {
                      return pathplanner::PathPlannerAuto("3 to Shoot").ToPtr().OnlyIf([] {
                        return SubArm::GetInstance().CheckIfArmHasGamePiece();
                      });
                    }),
                    [] { return SubArm::GetInstance().CheckIfArmHasGamePiece(); }  // Selector bool
                    )),
                [] { return SubArm::GetInstance().CheckIfArmHasGamePiece(); });
          });
}
>>>>>>> 7e61b1293ac1da9021103f8be057e21ef6d0342f

frc2::CommandPtr SubAuto::SimpleAuto(){
    return RunOnce([this]{pathplanner::PathPlannerAuto("A Center to 2").ToPtr();})
    .AndThen(
        cmd::Either(
            pathplanner::PathPlannerAuto("A 2 to Shoot").ToPtr()
            , 
            pathplanner::PathPlannerAuto("A 2 to 1").ToPtr()
            ,
     [] {return SubArm::GetInstance().CheckIfArmHasGamePiece();}));
}
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
