// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/angle.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>
#include <iostream>
#include <frc2/command/commands.h>
#include <frc2/command/button/Trigger.h>
#include <frc/MathUtil.h>
#include <frc/shuffleboard/Shuffleboard.h>

#include "subsystems/SubAmp.h"
#include "utilities/ICSparkMax.h"
#include "RobotContainer.h"

using namespace frc2::cmd;

SubAmp::SubAmp(){ 
    // amp shooter
    _ampMotorSpin.RestoreFactoryDefaults(); 

    // arm
    _armMotor.SetInverted(true);
    _armMotor.SetConversionFactor(1 / ARM_GEAR_RATIO);
    _armMotor.SetPIDFF(ARM_P, ARM_I, ARM_D, ARM_F);
    _armMotor.ConfigSmartMotion(ARM_MAX_VEL, ARM_MAX_ACCEL, ARM_TOLERANCE);

    _armMotorFollow.Follow(_armMotor);
    _armMotor.SetInverted(true);
    _armMotorFollow.SetInverted(true);
    _armMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    _armMotorFollow.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

// This method will be called once per scheduler run
void SubAmp::Periodic(){

    frc::SmartDashboard::PutData("amp/Arm Mechanism Display", &_doubleJointedArmMech);
    frc::SmartDashboard::PutNumber("amp/Amp Shooter Motor: ", _ampMotorSpin.Get());

    // angle of motor
    frc::SmartDashboard::PutData("amp/Dizzy Arm tilt motor: ", (wpi::Sendable*)&_armMotor);
    _arm1Ligament->SetAngle(_armMotor.GetPosition());
}

void SubAmp::SimulationPeriodic(){
    _armSim.SetInputVoltage(_armMotor.GetSimVoltage());
    _armSim.Update(20_ms);
    
    auto armAngle = _armSim.GetAngle();
    auto armVel = _armSim.GetVelocity();
    _armMotor.UpdateSimEncoder(-armAngle, -armVel);
}

// Shooter Amp
frc2::CommandPtr SubAmp::AmpShooter(){
   return StartEnd(
    [this]{_ampMotorSpin.Set(0.7);},
    [this]{_ampMotorSpin.Set(0);}
   );
}

frc2::CommandPtr SubAmp::ReverseAmpShooter(){
    return StartEnd(
        [this]{_ampMotorSpin.Set(-0.7);},
        [this]{_ampMotorSpin.Set(0);}
    );
}

// arm
frc2::CommandPtr SubAmp::TiltArmToAngle(units::degree_t targetAngle){ 
    return Run( [this, targetAngle]{_armMotor.SetSmartMotionTarget(targetAngle);}); 
}