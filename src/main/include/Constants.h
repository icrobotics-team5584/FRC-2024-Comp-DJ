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
namespace canid {
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
    // shooter amp
    constexpr int AmpMotorSpin = 13;
    
    // arm   
    constexpr int ArmMotor = 102;
    constexpr int ArmMotorFollow = 103;
}

namespace dio {
  constexpr int FDLineBreak = 100;
  constexpr int SDLineBreak = 101;
}

namespace pwm {

}