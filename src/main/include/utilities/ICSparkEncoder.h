#pragma once

#include <rev/CANSparkMax.h>

class ICSparkEncoder {
 public:
  ICSparkEncoder(rev::SparkRelativeEncoder&& inbuilt, rev::SparkAbsoluteEncoder&& absolute /*, rev::SparkMaxAlternateEncoder&& alternate*/ /*BRING ME BACK*/);
  double _absoluteSimPos = 0;
  enum EncoderType { INBUILT, ABSOLUTE, ALTERNATE };
  EncoderType selected = INBUILT;
  rev::SparkRelativeEncoder _inbuilt;
  rev::SparkAbsoluteEncoder _absolute;
  /*rev::SparkMaxAlternateEncoder _alternate; */ /*BRING ME BACK*/

  double GetPosition();
  double GetVelocity();
  void SetPosition(double pos);
  void SetConversionFactor(double rotationsToDesired);
  rev::SparkRelativeEncoder& GetInbuilt();
  rev::SparkAbsoluteEncoder& GetAbsolute();
  /* rev::SparkMaxAlternateEncoder& GetAlternate(); */ /*BRING ME BACK*/
};