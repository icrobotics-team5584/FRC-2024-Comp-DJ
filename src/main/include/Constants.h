#pragma once

namespace canivore {
  // 1-12 inclusive

}

namespace canid {
// amp
  constexpr int clawMotorJoint = 21;
  constexpr int elevatorMotor = 22;
  constexpr int ShooterMotorMain = 23;        // top motor
  constexpr int SecondaryShooterMotor = 24;  // bottom motor

  // shooter amp
  constexpr int AmpMotor = 25;
  constexpr int ShooterFeederMotor = 26;  // Shooter feeder

  // arm
  constexpr int ArmMotor = 27;
  constexpr int ArmMotorFollow = 28;
  constexpr int IntakeMotor = 29;

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

  //Climber
  constexpr int ClimberLeftMotor = 31;  // Not set up yet
  constexpr int ClimberRightMotor = 32; // Not set up yet
}

namespace pcm0 {
    constexpr int IntakeExtend = 2;
    constexpr int IntakeRetract = 3;
    constexpr int ShootClose = 4;
    constexpr int ShootFar = 5;
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

namespace pcm {
  constexpr int LockCylinderForward = 1;
  constexpr int LockCylinderReverse = 2;
}