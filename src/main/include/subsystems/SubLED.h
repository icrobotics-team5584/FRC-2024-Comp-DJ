#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/AddressableLED.h>
#include <array>
#include <constants.h>
#include <frc2/command/commands.h>
#include <rev/CANSparkMax.h>
#include <frc/motorcontrol/PWMSparkMax.h>

class SubLED : public frc2::SubsystemBase {
 public:
  SubLED();

  static SubLED& GetInstance() {
    static SubLED inst;
    return inst;
  }

  void SetLEDFunc(double PWMSIGNAL);
  frc2::CommandPtr SetLEDCommand(double PWMSIGNAL);
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  frc::PWMSparkMax _blinkin{pwm::LEDS};
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};