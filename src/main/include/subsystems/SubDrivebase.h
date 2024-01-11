// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <AHRS.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/controller/HolonomicDriveController.h>
#include <numbers>
#include <frc2/command/CommandPtr.h>
#include "Constants.h"
#include "utilities/SwerveModule.h"
#include <frc2/command/button/CommandXboxController.h>

class SubDrivebase : public frc2::SubsystemBase {
 public:
  static SubDrivebase& GetInstance() {
    static SubDrivebase inst;
    return inst;
  }
  SubDrivebase();
  void Periodic() override;
  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::degrees_per_second_t rot,
             bool fieldRelative);
  void AddVisionMeasurement(frc::Pose2d pose, double ambiguity,
                            units::second_t timeStamp);
  void ResetGyroHeading(units::degree_t startingAngle = 0_deg);
  void UpdatePosition(frc::Pose2d robotPosition);
  void DriveToPose(frc::Pose2d targetPose);
  bool IsAtPose(frc::Pose2d pose);
  void DisplayTrajectory(std::string name, frc::Trajectory trajectory);
  void SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue mode);
  void SetPose(frc::Pose2d pose);
  void DisplayPose(std::string label, frc::Pose2d pose);
  void UpdateOdometry();
  void SyncSensors();
  
  units::degree_t GetPitch();
  frc::Pose2d GetPose();
  frc::Rotation2d GetHeading();
  units::meters_per_second_t GetVelocity();
  frc::SwerveDriveKinematics<4> GetKinematics();
  frc::ChassisSpeeds GetRobotRelativeSpeeds();

  static constexpr units::meters_per_second_t MAX_VELOCITY = 3_mps;
  static constexpr units::degrees_per_second_t MAX_ANGULAR_VELOCITY =
      180_deg_per_s;
  static constexpr units::radians_per_second_squared_t MAX_ANGULAR_ACCEL{
      std::numbers::pi};

//Commands
frc2::CommandPtr JoystickDrive(frc2::CommandXboxController& controller);






 private:

  AHRS _gyro{frc::SerialPort::kMXP};

  frc::Translation2d _frontLeftLocation{+0.281_m, +0.281_m};
  frc::Translation2d _frontRightLocation{+0.281_m, -0.281_m};
  frc::Translation2d _backLeftLocation{-0.281_m, +0.281_m};
  frc::Translation2d _backRightLocation{-0.281_m, -0.281_m};

  const double FRONT_LEFT_MAG_OFFSET = -0.127930 ;
  const double FRONT_RIGHT_MAG_OFFSET = -0.198730;
  const double BACK_LEFT_MAG_OFFSET = -0.331543;  
  const double BACK_RIGHT_MAG_OFFSET = -0.467041; 

  SwerveModule _frontLeft{canivore::DriveBaseFrontLeftDrive, canivore::DriveBaseFrontLeftTurn, canivore::DriveBaseFrontLeftEncoder, FRONT_LEFT_MAG_OFFSET};
  SwerveModule _frontRight{canivore::DriveBaseFrontRightDrive, canivore::DriveBaseFrontRightTurn, canivore::DriveBaseFrontRightEncoder, FRONT_RIGHT_MAG_OFFSET};
  SwerveModule _backLeft{canivore::DriveBaseBackLeftDrive, canivore::DriveBaseBackLeftTurn, canivore::DriveBaseBackLeftEncoder, BACK_LEFT_MAG_OFFSET};
  SwerveModule _backRight{canivore::DriveBaseBackRightDrive, canivore::DriveBaseBackRightTurn, canivore::DriveBaseBackRightEncoder, BACK_RIGHT_MAG_OFFSET};

  frc::SwerveDriveKinematics<4> _kinematics{
      _frontLeftLocation, _frontRightLocation, _backLeftLocation,
      _backRightLocation};

  frc::PIDController Xcontroller{0.5,0,0};
  frc::PIDController Ycontroller{0.5,0,0};
  frc::ProfiledPIDController<units::radian> Rcontroller{1.8,0,0,{MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACCEL}};
  frc::HolonomicDriveController _driveController{Xcontroller, Ycontroller, Rcontroller};

  frc::SwerveDrivePoseEstimator<4> _poseEstimator{
      _kinematics, _gyro.GetRotation2d(), {frc::SwerveModulePosition{0_m, _frontLeft.GetAngle()}, 
				  frc::SwerveModulePosition{0_m, _frontRight.GetAngle()}, frc::SwerveModulePosition{0_m, _backLeft.GetAngle()}, 
          frc::SwerveModulePosition{0_m, _backRight.GetAngle()}} ,frc::Pose2d()
  };

  frc::Field2d _fieldDisplay;
  frc::Pose2d _prevPose; // Used for velocity calculations
};
