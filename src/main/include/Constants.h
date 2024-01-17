#pragma once

namespace canivore {
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
}

namespace canid {
    //amp
    constexpr int clawMotorJoint = 100;
    constexpr int elevatorMotor = 101;
    constexpr int ShooterMotorMain = 22; // top motor
    constexpr int SecondaryShooterMotor = 4; // bottom motor
    
}

namespace pcm0 {
    constexpr int IntakeExtend = 2;
    constexpr int IntakeRetract = 3;
    constexpr int ShootClose = 4;
    constexpr int ShootFar = 5;
    constexpr int Pcm0Id = 0;
}

namespace dio {

}

namespace pwm {

}


namespace OperatorConstants {

constexpr int kDriverControllerPort = 0;

}  // namespace OperatorConstants

