#pragma once

namespace canivore {
  // 1-12 inclusive

}

namespace canid {

  //drivebase IDS used: 1-12
 constexpr int DriveBaseFrontRightDrive = 7;
 constexpr int DriveBaseFrontRightTurn = 8;
 constexpr int DriveBaseFrontRightEncoder = 10;

 constexpr int DriveBaseFrontLeftDrive = 3;
 constexpr int DriveBaseFrontLeftTurn = 6;
 constexpr int DriveBaseFrontLeftEncoder = 9;
      
 constexpr int DriveBaseBackRightDrive = 5;
 constexpr int DriveBaseBackRightTurn = 2;
 constexpr int DriveBaseBackRightEncoder = 12;

 constexpr int DriveBaseBackLeftDrive = 1;
 constexpr int DriveBaseBackLeftTurn = 4;
 constexpr int DriveBaseBackLeftEncoder = 11;


//Intake IDS used: 13
  constexpr int IntakeMotor = 13;

//Shooter IDS used: 14-16
  constexpr int ShooterMotorMain = 14;
  constexpr int SecondaryShooterMotor = 15;
  constexpr int ShooterFeederMotor = 16;

//Arm IDS used: 17-18
  constexpr int ArmMotor = 17;

//Amp/Trap IDS used: 19
  constexpr int AmpMotor = 19;

//Climber IDS used: 20-21
  constexpr int lClimbMotor = 20;
  constexpr int rClimbMotor = 21;

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