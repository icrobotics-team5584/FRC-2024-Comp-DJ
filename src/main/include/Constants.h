#pragma once

namespace canivore {
  // 1-12 inclusive

}

namespace canid {

  //drivebase IDS used: 1-12
 constexpr int DriveBaseFrontRightDrive = 7; //done
 constexpr int DriveBaseFrontRightTurn = 8; //done
 constexpr int DriveBaseFrontRightEncoder = 10; //done

 constexpr int DriveBaseFrontLeftDrive = 3; //done
 constexpr int DriveBaseFrontLeftTurn = 6; //done
 constexpr int DriveBaseFrontLeftEncoder = 9; //done
      
 constexpr int DriveBaseBackRightDrive = 5; //done
 constexpr int DriveBaseBackRightTurn = 2; //done
 constexpr int DriveBaseBackRightEncoder = 12; //done

 constexpr int DriveBaseBackLeftDrive = 1; //done 
 constexpr int DriveBaseBackLeftTurn = 4; //done
 constexpr int DriveBaseBackLeftEncoder = 11; //done


//Intake IDS used: 13
  constexpr int IntakeMotor = 13; //17 robot

//Shooter IDS used: 14-16
  constexpr int ShooterMotorMain = 14; //18 robot
  constexpr int SecondaryShooterMotor = 15; //16 robot
  constexpr int ShooterFeederMotor = 16; //14 robot

//Arm IDS used: 17-18
  constexpr int ArmMotor = 17; //19

//Amp/Trap IDS used: 19
  constexpr int AmpMotor = 19; //20 robot

//Climber IDS used: 20-21
  constexpr int lClimbMotor = 20; //15 robot
  constexpr int rClimbMotor = 21; //13 robot

}

namespace pcm0 {
     //Intake
    constexpr int IntakeExtend = 0;
    constexpr int IntakeRetract = 1;

    //Shooter
    constexpr int ShootClose = 3;
    constexpr int ShootFar = 2;

    //Climber
    constexpr int LockCylinderForward = 4;
    constexpr int LockCylinderReverse = 5;

    constexpr int Pcm0Id = 0;

}

namespace pwm {
  constexpr int LEDS = 1;
}


namespace OperatorConstants {}

constexpr int kDriverControllerPort = 0;

namespace dio {
  constexpr int FDLineBreak = 0;
  constexpr int SDLineBreak = 1;
  constexpr int IntakeRetractedReed = 2;
  constexpr int IntakeExtendedReed = 3;
  constexpr int ArmHomeSwitch = 4;
}