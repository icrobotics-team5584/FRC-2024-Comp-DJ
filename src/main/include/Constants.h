<<<<<<< HEAD
#pragma once

namespace canivore {
  constexpr int DriveBaseFrontLeftDrive = 7;
  constexpr int DriveBaseFrontLeftTurn = 8;
  constexpr int DriveBaseFrontLeftEncoder = 10;

  constexpr int DriveBaseFrontRightDrive = 5;
  constexpr int DriveBaseFrontRightTurn = 6;
  constexpr int DriveBaseFrontRightEncoder = 12;

  constexpr int DriveBaseBackLeftDrive = 1;
  constexpr int DriveBaseBackLeftTurn = 2;
  constexpr int DriveBaseBackLeftEncoder = 11;

  constexpr int DriveBaseBackRightDrive = 3;
  constexpr int DriveBaseBackRightTurn = 4;
  constexpr int DriveBaseBackRightEncoder = 9;
}

namespace canid {
    //amp
    constexpr int ClawMotorJoint = 100;
    constexpr int ElevatorMotor = 101;
    constexpr int AmpMotorSpin = 13;
    constexpr int MotorForTilt = 102;
    constexpr int ExtraMotorForAmpShooter = 103;
    
}

namespace dio {

}

namespace pwm {

}
=======
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */
 // 2 4 6 8
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

namespace OperatorConstants {

constexpr int kDriverControllerPort = 0;

}  // namespace OperatorConstants

>>>>>>> main
