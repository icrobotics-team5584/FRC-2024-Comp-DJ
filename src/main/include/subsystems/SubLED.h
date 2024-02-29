#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/AddressableLED.h>
#include <array>
#include <constants.h>
#include <frc2/command/commands.h>

class SubLED : public frc2::SubsystemBase {
 public:
  frc2::CommandPtr IndicateSourceDrop();
  SubLED();

  static SubLED& GetInstance() {
    static SubLED inst;
    return inst;
  }

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  static constexpr int kLength = 31;

  frc::AddressableLED _led{pwm::LEDS};
  std::array<frc::AddressableLED::LEDData, kLength> m_ledBuffer;

  int firstPixelHue = 0;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};