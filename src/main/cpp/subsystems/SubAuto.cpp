// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubAuto.h"
#include <choreo/lib/Choreo.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <frc2/command/CommandPtr.h>
#include "subsystems/SubArm.h"
#include "subsystems/SubDrivebase.h"


using namespace frc2;

SubAuto::SubAuto() = default;

// This method will be called once per scheduler run
void SubAuto::Periodic() {}


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
// Description of total auton


// frc2::CommandPtr SubAuto::CloseNotesAuto() {
//   // Go from start pos to note 1
//   return pathplanner::PathPlannerAuto("Center to 1").ToPtr()
//       .AndThen(
//           // Either: go from note 1 to subwoofer or go from note 1 to note 2
          
//               cmd::Either(
//                 // Go from note 1 to subwoofer and shoot
//                 pathplanner::PathPlannerAuto("1 to Shoot")
//                     .ToPtr()
//                     .AndThen(
//                       // Then go from subwoofer to note 2
//                       pathplanner::PathPlannerAuto("Shoot to 2").ToPtr()
//                     )
//                     .AndThen(
//                       // Either: go from note 2 to subwoofer or go from note 2 to note 3
//                       cmd::Either(
//                           // Either:go from note 2 to subwoofer and shoot. END AUTO or go from 2 to
//                           // 3
//                           pathplanner::PathPlannerAuto("2 to Shoot").ToPtr(),
//                           pathplanner::PathPlannerAuto("2 to 3").ToPtr().AndThen(
//                               pathplanner::PathPlannerAuto("3 to Shoot").ToPtr().OnlyIf([] {
//                               return SubArm::GetInstance().CheckIfArmHasGamePiece();
//                             }));
//                           ),
//                           [] { return SubArm::GetInstance().CheckIfArmHasGamePiece(); });
//                     ),
//                 pathplanner::PathPlannerAuto("1 to 2").ToPtr().AndThen(cmd::Either(
//                     pathplanner::PathPlannerAuto("2 to Shoot").ToPtr().AndThen(
//                     pathplanner::PathPlannerAuto("Shoot to 3").ToPtr().OnlyIf([] {
//                         return SubArm::GetInstance().CheckIfArmHasGamePiece();
//                       })
//                     ),
//                     pathplanner::PathPlannerAuto("2 to 3").ToPtr().AndThen(
//                       return pathplanner::PathPlannerAuto("3 to Shoot").ToPtr().OnlyIf([] {
//                         return SubArm::GetInstance().CheckIfArmHasGamePiece();
//                       });
//                     ),
//                     [] { return SubArm::GetInstance().CheckIfArmHasGamePiece(); }  // Selector bool
//                     )),
//                 [] { return SubArm::GetInstance().CheckIfArmHasGamePiece(); });
//           );
// }


frc2::CommandPtr SubAuto::CloseNotesAuto(){

    return _centerTo1
    .AndThen(
        cmd::Either(
            _1ToShoot.AndThen(_shootTo2).AndThen(SubAuto::GetInstance().CNA1())
            , 

            _1To2.AndThen(SubAuto::GetInstance().CNA1())
            , 
     [] {return SubArm::GetInstance().CheckIfArmHasGamePiece();} ));
}  

frc2::CommandPtr SubAuto::CNA1(){

    return cmd::Either(
            _2ToShoot
            .AndThen(_shootTo3).AndThen(SubAuto::GetInstance().CNA2())
            , 
            _2To3.AndThen(SubAuto::GetInstance().CNA2())
            , 
     [] {return SubArm::GetInstance().CheckIfArmHasGamePiece();});
}


frc2::CommandPtr SubAuto::CNA2(){

    return cmd::Either(
            _3ToShoot
            .AndThen(_shootTo4).AndThen(SubAuto::GetInstance().CNA3())
            , 
            _3To4.AndThen(SubAuto::GetInstance().CNA3())
            , 
     [] {return SubArm::GetInstance().CheckIfArmHasGamePiece();});

}

frc2::CommandPtr SubAuto::CNA3(){

    return cmd::Either(
            _4ToShoot.AndThen(_shootTo5)
            , 
            _4To5
            , 
     [] {return SubArm::GetInstance().CheckIfArmHasGamePiece();});

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
