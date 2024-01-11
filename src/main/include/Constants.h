#pragma once

namespace canivore {
  // 1-12 inclusive
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
    // shooter amp
    constexpr int AmpMotorSpin = 13;

    // dizzy amp
    constexpr int ClawMotorJoint = 100;
    constexpr int ElevatorMotor = 101;
    
    // arm   
    constexpr int ArmMotor = 102;
    constexpr int ArmMotorFollow = 103;
}

namespace dio {

}

namespace pwm {

}