// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>
#include <frc/RobotBase.h>
#include <units/time.h>
#include <frc/DriverStation.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <frc2/command/Commands.h>
#include "subsystems/SubDrivebase.h"

SubDrivebase::SubDrivebase() {
  _gyro.Calibrate();
  Rcontroller.EnableContinuousInput(-180_deg, 180_deg);
  frc::SmartDashboard::PutData("field", &_fieldDisplay);

  using namespace pathplanner;
  AutoBuilder::configureHolonomic(
      [this]() { return GetPose(); },  // Robot pose supplier
      [this](frc::Pose2d pose) { SetPose(pose); },  // Method to reset odometry (will be called if your auto has a starting pose)
      [this]() { return GetRobotRelativeSpeeds(); },  // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      [this](frc::ChassisSpeeds speeds) { Drive(speeds.vx, speeds.vy, speeds.omega, false); },  // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      HolonomicPathFollowerConfig(
          PIDConstants(5.0, 0.0, 0.0),  // Translation PID constants
          PIDConstants(5.0, 0.0, 0.0),  // Rotation PID constants
          4.5_mps,                      // Max module speed, in m/s
          0.4_m,  // Drive base radius in meters. Distance from robot center to furthest module.
                  // NEEDS TO BE CHECKED AND MADE ACCURATE!!
          ReplanningConfig()  // Default path replanning config. See the API for the options here
          ),
      []() {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE 
        auto alliance = frc::DriverStation::GetAlliance();
        if (alliance) {
          return alliance.value() == frc::DriverStation::Alliance::kRed;
        }
        return false;
      },
      this  // Reference to this subsystem to set requirements
  );
}

// This method will be called once per scheduler run
void SubDrivebase::Periodic() {
  auto loopStart = frc::GetTime();
  // Dashboard Displays:
  frc::SmartDashboard::PutNumber("drivebase/heading", GetHeading().Degrees().value());
  frc::SmartDashboard::PutNumber("Drivebase/velocity", GetVelocity().value());

  frc::SmartDashboard::PutNumberArray("drivebase/true swerve states",
                                      std::array{
                                          _frontLeft.GetAngle().Degrees().value(),
                                          _frontLeft.GetSpeed().value(),
                                          _frontRight.GetAngle().Degrees().value(),
                                          _frontRight.GetSpeed().value(),
                                          _backLeft.GetAngle().Degrees().value(),
                                          _backLeft.GetSpeed().value(),
                                          _backRight.GetAngle().Degrees().value(),
                                          _backRight.GetSpeed().value(),
                                      });

  _frontLeft.SendSensorsToDash();
  _frontRight.SendSensorsToDash();
  _backLeft.SendSensorsToDash();
  _backRight.SendSensorsToDash();

  UpdateOdometry();
  frc::SmartDashboard::PutNumber("drivebase/loop time (sec)", (frc::GetTime() - loopStart).value());
}

frc2::CommandPtr SubDrivebase::JoystickDrive(frc2::CommandXboxController& controller) {
  return Run([this, &controller] {
    auto forwardSpeed = controller.GetLeftY() * MAX_VELOCITY;
    auto rotationSpeed = controller.GetRightX() * MAX_ANGULAR_VELOCITY;
    auto sidewaysSpeed = controller.GetLeftX() * MAX_VELOCITY;
    Drive(forwardSpeed, sidewaysSpeed, rotationSpeed, true);
  });
}

void SubDrivebase::Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed,
                         units::degrees_per_second_t rot, bool fieldRelative) {
  // Get states of all swerve modules
  auto states = _kinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, GetHeading())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  // Set speed limit and apply speed limit to all modules
  _kinematics.DesaturateWheelSpeeds(&states, MAX_VELOCITY);

  // Setting modules from aquired states
  auto [fl, fr, bl, br] = states;

  frc::SmartDashboard::PutNumberArray("drivebase/desired swerve states",
                                      std::array{
                                          fl.angle.Degrees().value(),
                                          fl.speed.value(),
                                          fr.angle.Degrees().value(),
                                          fr.speed.value(),
                                          bl.angle.Degrees().value(),
                                          bl.speed.value(),
                                          br.angle.Degrees().value(),
                                          br.speed.value(),
                                      });

  _frontLeft.SetDesiredState(fl);
  _frontRight.SetDesiredState(fr);
  _backLeft.SetDesiredState(bl);
  _backRight.SetDesiredState(br);

  // Check if robot is in simulation.
  // Manualy adjusting gyro by calculating rotation in simulator as gyro is not enabled in
  // simulation
  if (frc::RobotBase::IsSimulation()) {
    units::radian_t radPer20ms = rot * 20_ms;
    units::degree_t newHeading = GetHeading().RotateBy(radPer20ms).Degrees();
    _gyro.SetAngleAdjustment(-newHeading.value());  // negative to switch to CW from CCW
  }
}

frc::ChassisSpeeds SubDrivebase::GetRobotRelativeSpeeds() {
  auto fl = _frontLeft.GetState();
  auto fr = _frontRight.GetState();
  auto bl = _backLeft.GetState();
  auto br = _backRight.GetState();
  return _kinematics.ToChassisSpeeds(fl, fr, bl, br);
}

// Syncs encoder values when the robot is turned on
void SubDrivebase::SyncSensors() {
  _frontLeft.SyncSensors();
  _frontRight.SyncSensors();
  _backLeft.SyncSensors();
  _backRight.SyncSensors();
  _gyro.Calibrate();
}

frc2::CommandPtr SubDrivebase::SyncSensorBut(){
  return RunOnce([this]{SyncSensors();});
}

frc::Rotation2d SubDrivebase::GetHeading() {
  return _gyro.GetRotation2d();
}

// Calculate robot's velocity over past time step (20 ms)
units::meters_per_second_t SubDrivebase::GetVelocity() {
  auto robotDisplacement =
      _prevPose.Translation().Distance(_poseEstimator.GetEstimatedPosition().Translation());
  return units::meters_per_second_t{robotDisplacement / 20_ms};
}

frc::SwerveDriveKinematics<4> SubDrivebase::GetKinematics() {
  return _kinematics;
}

// calculates the relative field location
void SubDrivebase::UpdateOdometry() {
  auto fl = _frontLeft.GetPosition();
  auto fr = _frontRight.GetPosition();
  auto bl = _backLeft.GetPosition();
  auto br = _backRight.GetPosition();

  _prevPose = _poseEstimator.GetEstimatedPosition();
  _poseEstimator.Update(GetHeading(), {fl, fr, bl, br});
  _fieldDisplay.SetRobotPose(_poseEstimator.GetEstimatedPosition());
}

void SubDrivebase::DriveToPose(frc::Pose2d targetPose) {
  DisplayPose("targetPose", targetPose);

  frc::Pose2d currentPosition = _poseEstimator.GetEstimatedPosition();
  double speedX = Xcontroller.Calculate(currentPosition.X().value(), targetPose.X().value());
  double speedY = Ycontroller.Calculate(currentPosition.Y().value(), targetPose.Y().value());
  double speedRot =
      Rcontroller.Calculate(currentPosition.Rotation().Radians(), targetPose.Rotation().Radians());

  speedX = std::clamp(speedX, -0.5, 0.5);
  speedY = std::clamp(speedY, -0.5, 0.5);
  speedRot = std::clamp(speedRot, -2.0, 2.0);

  // Drive speeds are relative to your alliance wall. Flip if we are on red,
  // since we are using global coordinates (blue alliance at 0,0)
  if (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed && frc::RobotBase::IsReal()) {
    Drive(-speedX * 1_mps, -speedY * 1_mps, speedRot * 1_rad_per_s, true);
  } else {
    Drive(speedX * 1_mps, speedY * 1_mps, speedRot * 1_rad_per_s, true);
  }
}

bool SubDrivebase::IsAtPose(frc::Pose2d pose) {
  auto currentPose = _poseEstimator.GetEstimatedPosition();
  auto rotError = currentPose.Rotation() - pose.Rotation();
  auto posError = currentPose.Translation().Distance(pose.Translation());

  if (units::math::abs(rotError.Degrees()) < 1_deg && posError < 1_cm) {
    return true;
  } else {
    return false;
  }
}

void SubDrivebase::ResetGyroHeading(units::degree_t startingAngle) {
  _gyro.Reset();
  _gyro.SetAngleAdjustment(startingAngle.value());
}

frc2::CommandPtr SubDrivebase::ResetGyroCmd(){
  return RunOnce([this]{ResetGyroHeading();});
}

frc::Pose2d SubDrivebase::GetPose() {
  return _poseEstimator.GetEstimatedPosition();
}

void SubDrivebase::SetPose(frc::Pose2d pose) {
  auto fl = _frontLeft.GetPosition();
  auto fr = _frontRight.GetPosition();
  auto bl = _backLeft.GetPosition();
  auto br = _backRight.GetPosition();
  _poseEstimator.ResetPosition(GetHeading(), {fl, fr, bl, br}, pose);
}

void SubDrivebase::DisplayPose(std::string label, frc::Pose2d pose) {
  _fieldDisplay.GetObject(label)->SetPose(pose);
}

void SubDrivebase::UpdatePosition(frc::Pose2d robotPosition) {
  _poseEstimator.AddVisionMeasurement(robotPosition, 2_ms);
}

void SubDrivebase::DisplayTrajectory(std::string name, frc::Trajectory trajectory) {
  _fieldDisplay.GetObject(name)->SetTrajectory(trajectory);
}

void SubDrivebase::AddVisionMeasurement(frc::Pose2d pose, double ambiguity,
                                        units::second_t timeStamp) {
  frc::SmartDashboard::PutNumber("Timestamp", timeStamp.value());
  _poseEstimator.AddVisionMeasurement(pose, timeStamp);
}

void SubDrivebase::SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue mode) {
  _frontLeft.SetNeutralMode(mode);
  _frontRight.SetNeutralMode(mode);
  _backLeft.SetNeutralMode(mode);
  _backRight.SetNeutralMode(mode);
}

units::degree_t SubDrivebase::GetPitch() {
  return _gyro.GetPitch() * 1_deg;
}