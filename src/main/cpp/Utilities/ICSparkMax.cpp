#include "utilities/ICSparkMax.h"

#include <frc/RobotBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/voltage.h>
#include <cstdlib>
#include <iostream>

ICSparkMax::ICSparkMax(int deviceID, units::ampere_t currentLimit)
    : rev::CANSparkMax(deviceID, rev::CANSparkLowLevel::MotorType::kBrushless) {
  RestoreFactoryDefaults();
  SetSmartCurrentLimit(currentLimit.value());
  SetConversionFactor(1);  // Makes the internal encoder use revs per sec not revs per min

  _pidController.SetSmartMotionMinOutputVelocity(0);
  SetClosedLoopOutputRange(-1, 1);
}

void ICSparkMax::InitSendable(wpi::SendableBuilder& builder) {
  // clang-format off
  builder.AddDoubleProperty("Position", [&] { return GetPosition().value(); }, nullptr);  // setter is null, cannot set position directly
  builder.AddDoubleProperty("Velocity", [&] { return GetVelocity().value(); }, nullptr);
  builder.AddDoubleProperty("Position Target", [&] { return GetPositionTarget().value(); }, [&](double targ) { SetPositionTarget(targ*1_tr); });
  builder.AddDoubleProperty("Velocity Target", [&] { return GetVelocityTarget().value(); }, [&](double targ) { SetVelocityTarget(targ*1_tps); });

  builder.AddDoubleProperty("Voltage", [&] { 
        return (frc::RobotBase::IsSimulation()) 
          ? GetSimVoltage().value() 
          : CANSparkMax::GetAppliedOutput() * 12;
      }, nullptr);

  builder.AddDoubleProperty("P Gain", [&] { return _simController.GetP(); }, [&](double P) { SetP(P); });
  builder.AddDoubleProperty("I Gain", [&] { return _simController.GetI(); }, [&](double I) { SetI(I); });
  builder.AddDoubleProperty("D Gain", [&] { return _simController.GetD(); }, [&](double D) { SetD(D); });
  builder.AddDoubleProperty("FF Gain", [&] { return _simFF; }, [&](double FF) { SetFF(FF); });
  // clang-format on
}

void ICSparkMax::SetPosition(units::turn_t position) {
  _encoder.SetPosition(position.value());
  auto err = GetLastError();
}

void ICSparkMax::SetPositionTarget(units::turn_t target, units::volt_t arbFeedForward) {
  _positionTarget = target;
  _velocityTarget = units::turns_per_second_t{0};
  _voltageTarget = 0_V;
  _arbFeedForward = arbFeedForward;
  SetInternalControlType(Mode::kPosition);

  _pidController.SetReference(target.value(), GetControlType(), 0, _arbFeedForward.value());
}

void ICSparkMax::SetSmartMotionTarget(units::turn_t target, units::volt_t arbFeedForward) {
  _positionTarget = target;
  _smartMotionProfileTimer.Start();
  _velocityTarget = units::turns_per_second_t{0};
  _voltageTarget = 0_V;
  _arbFeedForward = arbFeedForward;
  SetInternalControlType(Mode::kSmartMotion);

  _pidController.SetReference(target.value(), GetControlType(), 0, _arbFeedForward.value());
}

void ICSparkMax::SetVelocityTarget(units::turns_per_second_t target, units::volt_t arbFeedForward) {
  _velocityTarget = target;
  _positionTarget = units::turn_t{0};
  _voltageTarget = 0_V;
  _arbFeedForward = arbFeedForward;
  SetInternalControlType(Mode::kVelocity);

  _pidController.SetReference(target.value(), GetControlType(), 0, _arbFeedForward.value());
}

void ICSparkMax::Set(double speed) {
  _velocityTarget = units::turns_per_second_t{0};
  _positionTarget = units::turn_t{0};
  _voltageTarget = 0_V;
  SetInternalControlType(Mode::kDutyCycle);
  if (frc::RobotBase::IsSimulation()) {
    _pidController.SetReference(speed, Mode::kDutyCycle);
  }
  CANSparkMax::Set(speed);
}

void ICSparkMax::SetVoltage(units::volt_t output) {
  _velocityTarget = units::turns_per_second_t{0};
  _positionTarget = units::turn_t{0};
  _voltageTarget = output;
  _arbFeedForward = 0_V;
  SetInternalControlType(Mode::kVoltage);

  _pidController.SetReference(output.value(), GetControlType(), 0, _arbFeedForward.value());
}

void ICSparkMax::StopMotor() {
  _velocityTarget = units::turns_per_second_t{0};
  _positionTarget = units::turn_t{0};
  _voltageTarget = 0_V;
  SetInternalControlType(Mode::kDutyCycle);
  CANSparkMax::StopMotor();
}

void ICSparkMax::SetInternalControlType(Mode controlType) {
  _controlType = controlType;
  _simControlMode.Set((int)_controlType);
}

void ICSparkMax::ConfigSmartMotion(units::turns_per_second_t maxVelocity,
                                   units::turns_per_second_squared_t maxAcceleration,
                                   units::turn_t tolerance) {
  _pidController.SetSmartMotionMaxAccel(AccelToSparkRPMps(maxAcceleration));
  _pidController.SetSmartMotionMaxVelocity(VelToSparkRPM(maxVelocity));
  _pidController.SetSmartMotionAllowedClosedLoopError(tolerance.value());

  _simSmartMotionProfile = frc::TrapezoidProfile<units::turns>{{maxVelocity, maxAcceleration}};
}

void ICSparkMax::SetConversionFactor(double rotationsToDesired) {
  _encoder.SetPositionConversionFactor(rotationsToDesired);
  // Need to divide vel by 60 because Spark Max uses Revs per minute not Revs per second
  _encoder.SetVelocityConversionFactor(rotationsToDesired / 60);
}

void ICSparkMax::UseAlternateEncoder(int countsPerRev) {
  // const double posConversion = _encoder->GetPositionConversionFactor();

  // _encoder = std::make_unique<rev::SparkMaxAlternateEncoder>(
  //     CANSparkMax::GetAlternateEncoder(countsPerRev));
  // _pidController.SetFeedbackDevice(*_encoder);

  // SetConversionFactor(posConversion);
}

void ICSparkMax::UseAbsoluteEncoder(rev::SparkAbsoluteEncoder& encoder) {
  _pidController.SetFeedbackDevice(encoder);
}

void ICSparkMax::EnableClosedLoopWrapping(units::turn_t min, units::turn_t max) {
  _pidController.SetPositionPIDWrappingMinInput(min.value());
  _pidController.SetPositionPIDWrappingMaxInput(max.value());
  _pidController.SetPositionPIDWrappingEnabled(true);
  _simController.EnableContinuousInput(PosToSparkRevs(min), PosToSparkRevs(max));
}

void ICSparkMax::SetPIDFF(double P, double I, double D, double FF) {
  SetP(P);
  SetI(I);
  SetD(D);
  SetFF(FF);
}

void ICSparkMax::SetP(double P) {
  _pidController.SetP(P);
  _simController.SetP(P);
}

void ICSparkMax::SetI(double I) {
  _pidController.SetI(I);
  _simController.SetI(I);
}

void ICSparkMax::SetD(double D) {
  _pidController.SetD(D);
  _simController.SetD(D);
}

void ICSparkMax::SetFF(double FF) {
  _pidController.SetFF(FF);
  _simFF = FF;
}

void ICSparkMax::SetClosedLoopOutputRange(double minOutputPercent, double maxOutputPercent) {
  _minPidOutput = minOutputPercent;
  _maxPidOutput = maxOutputPercent;
  _pidController.SetOutputRange(minOutputPercent, maxOutputPercent);
}

units::turns_per_second_t ICSparkMax::GetVelocity() {
  if (frc::RobotBase::IsSimulation()) {
    return _simVelocity;
  } else {
    return units::turns_per_second_t{_encoder.GetVelocity()};
  }
}

units::volt_t ICSparkMax::GetSimVoltage() {
  units::volt_t output = 0_V;

  switch (_controlType) {
    case Mode::kDutyCycle:
      output = units::volt_t{CANSparkMax::Get() * 12};
      break;

    case Mode::kVelocity:
      output = units::volt_t{
          _simController.Calculate(VelToSparkRPM(GetVelocity()), VelToSparkRPM(_velocityTarget)) +
          _simFF * VelToSparkRPM(_velocityTarget)};
      break;

    case Mode::kPosition:
      output = units::volt_t{
          _simController.Calculate(PosToSparkRevs(GetPosition()), PosToSparkRevs(_positionTarget)) +
          _simFF * PosToSparkRevs(_positionTarget)};
      break;

    case Mode::kVoltage:
      output = _voltageTarget;
      break;

    case Mode::kSmartMotion:
      output = units::volt_t{_simController.Calculate(
          VelToSparkRPM(GetVelocity()),
          VelToSparkRPM(EstimateSMVelocity()) + _simFF * VelToSparkRPM(EstimateSMVelocity()))};
      break;

    case Mode::kCurrent:
      break;

    case Mode::kSmartVelocity:
      break;
  }
  output += _arbFeedForward;
  return std::clamp(output, _minPidOutput * 12_V, _maxPidOutput * 12_V);
}

void ICSparkMax::UpdateSimEncoder(units::turn_t position, units::turns_per_second_t velocity) {
  _encoder.SetPosition(position.value());
  _simVelocity = velocity;
}

units::turns_per_second_t ICSparkMax::EstimateSMVelocity() {
  if (_controlType != Mode::kSmartMotion) {
    return units::turns_per_second_t{0};
  }

  units::turn_t error = units::math::abs(_positionTarget - GetPosition());
  units::turn_t tolerance = SparkRevsToPos(_pidController.GetSmartMotionAllowedClosedLoopError());
  if (error < tolerance) {
    return units::turns_per_second_t{0};
  }

  return _simSmartMotionProfile
      .Calculate(20_ms, {_positionTarget, units::turns_per_second_t{0}},
                 {GetPosition(), GetVelocity()})
      .velocity;
}