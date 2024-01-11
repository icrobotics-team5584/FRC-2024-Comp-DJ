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

// tilts _clawMotorJoint
SubAmp::SubAmp(){ 
    // amp shooter
    _ampMotorSpin.RestoreFactoryDefaults(); 

    // clawMotorJoint
    _clawMotorJoint.SetInverted(true);
    _clawMotorJoint.SetConversionFactor(1 / CLAW_GEAR_RATIO);
    _clawMotorJoint.SetPIDFF(CLAW_P, CLAW_I, CLAW_D, CLAW_F);
    _clawMotorJoint.ConfigSmartMotion(CLAW_MAX_VEL, CLAW_MAX_ACCEL, CLAW_TOLERANCE);
    _clawMotorJoint.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    // arm
    _armMotor.SetInverted(true);
    _armMotor.SetConversionFactor(1 / CLAW_GEAR_RATIO);
    _armMotor.SetPIDFF(CLAW_P, CLAW_I, CLAW_D, CLAW_F);
    _armMotor.ConfigSmartMotion(CLAW_MAX_VEL, CLAW_MAX_ACCEL, CLAW_TOLERANCE);

    _armMotorFollow.Follow(_armMotor);
    _armMotor.SetInverted(true);
    _armMotorFollow.SetInverted(true);
    _armMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    _armMotorFollow.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

// This method will be called once per scheduler run
void SubAmp::Periodic(){

    frc::SmartDashboard::PutData("amp/Arm Mechanism Display", &_doubleJointedArmMech);
    frc::SmartDashboard::PutData("amp/Claw Mechanism Display", &_doubleJointedClawMech);
    frc::SmartDashboard::PutNumber("amp/Amp Shooter Motor: ", _ampMotorSpin.Get());
    frc::SmartDashboard::PutNumber("amp/Dizzy Claw tilt motor speed: ", _clawMotorJoint.Get());
    frc::SmartDashboard::PutNumber("amp/Dizzy Claw tilt motor speed: ", _clawMotorJoint.Get());

    // angle of motor
    frc::SmartDashboard::PutData("amp/Dizzy Claw tilt motor: ", (wpi::Sendable*)&_clawMotorJoint);
    _claw1Ligament->SetAngle(_clawMotorJoint.GetPosition());

    frc::SmartDashboard::PutData("amp/Dizzy Arm tilt motor: ", (wpi::Sendable*)&_armMotor);
    _arm1Ligament->SetAngle(_armMotor.GetPosition());
}

void SubAmp::SimulationPeriodic(){
    _clawSim.SetInputVoltage(_clawMotorJoint.GetSimVoltage());
    _clawSim.Update(20_ms);
    
    auto clawAngle = _clawSim.GetAngle();
    auto clawVel = _clawSim.GetVelocity();
    _clawMotorJoint.UpdateSimEncoder(-clawAngle, -clawVel);

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

// dizzy Amp
frc2::CommandPtr SubAmp::MotorTiltToAngle(units::degree_t targetAngle){ 
    return Run( [this, targetAngle]{_clawMotorJoint.SetSmartMotionTarget(targetAngle);}); 
}

// arm
frc2::CommandPtr SubAmp::TiltArmToAngle(units::degree_t targetAngle){ 
    return Run( [this, targetAngle]{_armMotor.SetSmartMotionTarget(targetAngle);}); 
}