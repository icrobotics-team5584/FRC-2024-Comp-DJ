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
    _ampMotorSpin.RestoreFactoryDefaults(); 
    _motorForTilt.SetInverted(true);
}

// This method will be called once per scheduler run
void SubAmp::Periodic(){
    frc::SmartDashboard::PutNumber("amp/Amp Shooter Motor: ", _ampMotorSpin.Get());
    frc::SmartDashboard::PutNumber("amp/Dizzy Claw tilt motor speed: ", _clawMotorJoint.Get());
    frc::SmartDashboard::PutNumber("amp/Dizzy Claw tilt motor speed: ", _motorForTilt.Get());


    // angle of motor
    frc::SmartDashboard::PutData("amp/Dizzy Claw tilt motor: ", (wpi::Sendable*)&_motorForTilt);
    _motorForTilt.SetConversionFactor(1 / GEAR_RATIO);
    _motorForTilt.SetPIDFF(P, I, D, F);
    _motorForTilt.ConfigSmartMotion(MAX_VEL, MAX_ACCEL, TOLERANCE);
    

}

void SubAmp::SimulationPeriodic(){
    _clawSim.SetInputVoltage(_motorForTilt.GetSimVoltage());
    _clawSim.Update(20_ms);
    
    auto clawAngle = _clawSim.GetAngle();
    auto clawVel = _clawSim.GetVelocity();
    _motorForTilt.UpdateSimEncoder(-clawAngle, -clawVel);

}

// drive motor to target angle

frc2::CommandPtr SubAmp::MotorTiltToAngle(units::degree_t targetAngle){ 
    return Run( [this, targetAngle]{_motorForTilt.SetSmartMotionTarget(targetAngle);} ); 
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
frc2::CommandPtr SubAmp::ClawTiltDown(){
    return StartEnd(
        [this]{_clawMotorJoint.Set(-0.5);},
        [this]{_clawMotorJoint.Set(-0.01);}
    );
}

frc2::CommandPtr SubAmp::ClawTiltUp(){
    return StartEnd(
        [this]{_clawMotorJoint.Set(0.5);},
        [this]{_clawMotorJoint.Set(0.01);}
    );
}