// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/angle.h>

#include "subsystems/SubAmp.h"
#include "utilities/ICSparkMax.h"

using namespace frc2::cmd;

SubAmp::SubAmp(){ _ampMotorSpin.RestoreFactoryDefaults(); }

// This method will be called once per scheduler run
void SubAmp::Periodic(){
    frc::SmartDashboard::PutNumber("amp/Amp Shooter Motor: ", _ampMotorSpin.Get());
    frc::SmartDashboard::PutNumber("amp/Dizzy Claw tilt motor speed: ", _clawMotorJoint.Get());

    /*
    // angle of motor
    frc::SmartDashboard::PutData("amp/Dizzy Claw tilt motor: ", (wpi::Sendable*)&_motorForTilt);
    _motorForTilt.SetConversionFactor(1 / GEAR_RATIO);
    _motorForTilt.SetPIDFF(P, I, D, F);
    _motorForTilt.ConfigSmartMotion(MAX_VEL, MAX_ACCEL, TOLERANCE);
    */

}

frc2::CommandPtr SubAmp::MotorTiltToAngle(){ 
    return RunOnce( 
        [this]{ _motorForTilt.SetSmartMotionTarget(10_deg); } 
    );
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

frc2::CommandPtr SubAmp::ExtraMotor(){
   return StartEnd(
    [this]{_ampMotorSpin.Set(0.7);},
    [this]{_ampMotorSpin.Set(0);}
   );
}

frc2::CommandPtr SubAmp::ReverseExtraMotor(){
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

/*
void SubArm::SimulationPeriodic() {
  _armSim.SetInputVoltage(_armMotorBottom.GetSimVoltage());
  _armSim.Update(20_ms);

  _armSim2.SetInputVoltage(_armMotorTop.GetSimVoltage());
  _armSim2.Update(20_ms);

  auto armAngle = _armSim.GetAngle();
  auto armVel = _armSim.GetVelocity();
  _armMotorBottom.UpdateSimEncoder(armAngle, armVel);

  auto armAngle2 = _armSim2.GetAngle();
  auto armVel2 = _armSim2.GetVelocity();
  _armMotorTop.UpdateSimEncoder(armAngle2, armVel2);
  */