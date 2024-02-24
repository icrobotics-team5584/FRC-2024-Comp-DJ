// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubLED.h"
#include <frc/AddressableLED.h>
#include "RobotContainer.h"
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

int R = std::rand() % 255;
int G = std::rand() % 255;
int B = std::rand() % 255;

using namespace frc2::cmd;

SubLED::SubLED() {
  _led.SetLength(kLength);
  _led.SetData(m_ledBuffer);
  _led.Start();
}
// This method will be called once per scheduler run
void SubLED::Periodic() {
  auto loopStart = frc::GetTime();
  frc::SmartDashboard::PutNumber("led/loop time (sec)", (frc::GetTime()-loopStart).value());
}

frc2::CommandPtr SubLED::IndicateSourceDrop() {
  return RunOnce([this] {
           for (auto& LED : m_ledBuffer) {
             LED.SetRGB(255, 0, 0);
           }
           _led.SetData(m_ledBuffer);
         })
      .AndThen(Wait(10_s).FinallyDo([this] {
        for (auto& LED : m_ledBuffer) {
          LED.SetRGB(0, 0, 0);
        }
        _led.SetData(m_ledBuffer);
      }));
}
