#include <Utilities/ICSparkEncoder.h>
#include <frc/RobotBase.h>
#include <utility>

ICSparkEncoder::ICSparkEncoder(rev::SparkRelativeEncoder&& inbuilt,
                               rev::SparkAbsoluteEncoder&& absolute
                              /*, rev::SparkMaxAlternateEncoder&& alternate*/ /*BRING ME BACK*/)
    : _inbuilt(std::move(inbuilt)),
      _absolute(std::move(absolute))
     /* _alternate(std::move(alternate))*/ {}  /*BRING ME BACK*/

double ICSparkEncoder::GetPosition() {
  switch (selected) {
    case ABSOLUTE:
      return frc::RobotBase::IsSimulation() ? _absoluteSimPos : _absolute.GetPosition();
    case ALTERNATE:
      /*return _alternate.GetPosition(); */ /*BRING ME BACK*/ return 0;
    case INBUILT:
    default:
      return _inbuilt.GetPosition();
  }
}

double ICSparkEncoder::GetVelocity() {
  switch (selected) {
    case ABSOLUTE:
      return _absolute.GetVelocity();
    case ALTERNATE:
     /* return _alternate.GetVelocity(); */ return 0;
    case INBUILT:
    default:
      return _inbuilt.GetVelocity();
  }
}

void ICSparkEncoder::SetPosition(double pos) {
  /*_alternate.SetPosition(pos); */ /*BRING ME BACK*/
  _inbuilt.SetPosition(pos);
  _absoluteSimPos = pos;
}

void ICSparkEncoder::SetConversionFactor(double rotationsToDesired) {
  // Need to divide vel by 60 because Spark Max uses Revs per minute not Revs per second
  _absolute.SetPositionConversionFactor(rotationsToDesired);
  _absolute.SetVelocityConversionFactor(rotationsToDesired / 60);
 /* _alternate.SetPositionConversionFactor(rotationsToDesired);
  _alternate.SetVelocityConversionFactor(rotationsToDesired / 60); */ /*BRING ME BACK*/
  _inbuilt.SetPositionConversionFactor(rotationsToDesired);
  _inbuilt.SetVelocityConversionFactor(rotationsToDesired / 60);
}

rev::SparkRelativeEncoder& ICSparkEncoder::GetInbuilt() {
  return _inbuilt;
}
rev::SparkAbsoluteEncoder& ICSparkEncoder::GetAbsolute() {
  return _absolute;
}
/*rev::SparkMaxAlternateEncoder& ICSparkEncoder::GetAlternate() {
  return _alternate;
} */ /*BRING ME BACK*/