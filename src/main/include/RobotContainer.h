// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/CommandGenericHID.h> 
#include <frc2/command/InstantCommand.h>
#include <frc/XboxController.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "Constants.h"
#include <frc/XboxController.h>
#include <frc2/command/button/CommandGenericHID.h>
#include <frc2/command/InstantCommand.h>
#include <frc/DigitalInput.h>

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/button/CommandXboxController.h>

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  void ConfigureBindings();
  frc2::CommandXboxController _driverController{0};

  frc::SendableChooser<std::string> _autoChooser;
  frc::SendableChooser<int> _delayChooser;
  std::string _autoSelected;
};
