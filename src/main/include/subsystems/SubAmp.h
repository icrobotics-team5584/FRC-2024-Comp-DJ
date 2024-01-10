// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc2/command/commands.h>
#include <units/mass.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>

#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/DigitalInput.h>
#include <frc/controller/ArmFeedforward.h>
#include <wpi/interpolating_map.h>
#include <networktables/NetworkTableEntry.h>

#include "Constants.h"
#include "utilities/ICSparkMax.h"





class SubAmp : public frc2::SubsystemBase {
 public:
  SubAmp();
  //void SimulationPeriodic() override;

  //Instance
  static SubAmp &GetInstance(){
    static SubAmp inst;
    return inst;
  }

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  frc2::CommandPtr MotorTiltToAngle();

  // shooter amp
  frc2::CommandPtr AmpShooter();
  frc2::CommandPtr ReverseAmpShooter();

  frc2::CommandPtr ExtraMotor();
  frc2::CommandPtr ReverseExtraMotor();

  // dizzy amp
  frc2::CommandPtr ClawTiltDown();
  frc2::CommandPtr ClawTiltUp();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  //units::turn_t TopArmAngleToEncoderAngle(units::turn_t topArmAngle);

  ICSparkMax _motorForTilt{canid::MotorForTilt};
  ICSparkMax _ampMotorSpin{canid::AmpMotorSpin};
  ICSparkMax _clawMotorJoint{canid::ClawMotorJoint};
  ICSparkMax _extraMotorForAmpShooter{canid::ExtraMotorForAmpShooter};

  // tilting a motor to an angle
  //arm 1
  static constexpr double P = 0.0;
  static constexpr double I = 0.0;
  static constexpr double D = 0.0;
  static constexpr double F = 15;

  // Bottom arm FF is all zeros, it will be dynamically set in Periodic() based
  // on the position of the top arm and the Gravity FF Map.
  
  wpi::interpolating_map<units::degree_t, units::volt_t> _bottomArmGravFFMap; 
  frc::ArmFeedforward _bottomArmGravityFF{0_V, 0.5_V, 0_V / 1_rad_per_s, 0_V / 1_rad_per_s_sq};
  
  static constexpr double GEAR_RATIO = 218.27;
  static constexpr units::kilogram_t MASS = 1_kg; // only sim
  static constexpr units::degrees_per_second_t MAX_VEL = 18_deg_per_s;
  static constexpr units::degrees_per_second_squared_t MAX_ACCEL = 90_deg_per_s_sq;
  static constexpr units::degree_t TOLERANCE = 0.5_deg; 
  static constexpr units::meter_t LENGTH = 0.9_m;
  static constexpr units::degree_t MIN_ANGLE = -180_deg; // only sim
  static constexpr units::degree_t MAX_ANGLE = 180_deg; // only sim

  // simulator

  // simulating claw
  
  // simulation of armMotor1
  /*
  frc::sim::SingleJointedArmSim _armSim{
    frc::DCMotor::NEO(2),
    GEAR_RATIO, 
    frc::sim::SingleJointedArmSim::EstimateMOI(LENGTH, MASS),
    LENGTH,
    MIN_ANGLE,
    MAX_ANGLE,
    false,
  };

  // Display of arm sim
  frc::Mechanism2d _doubleJointedArmMech{3, 3}; //canvas width and height
  frc::MechanismRoot2d* _root = _doubleJointedArmMech.GetRoot("armRoot", 1, 1); //root x and y
  frc::MechanismLigament2d* _arm1Ligament = _root->Append<frc::MechanismLigament2d>("ligament1", LENGTH.value(), 5_deg);
  frc::MechanismLigament2d* _arm2Ligament = _arm1Ligament->Append<frc::MechanismLigament2d>("ligament2", LENGTH.value(), 5_deg);

  nt::GenericEntry* _xoffset;
  nt::GenericEntry* _yoffset;
  */

};
