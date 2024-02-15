#pragma once

#include <filesystem>
#include <fstream>
#include <frc/smartdashboard/smartdashboard.h>
#include <string>

namespace BotVars {

enum Robot { COMP, PRACTICE };
const std::string COMP_BOT_MAC_ADDRESS = "00:15:5d:45:d7:5f";
const std::string PRACTICE_BOT_MAC_ADDRESS = "ff:ff:ff:ff:ff:ff";

Robot DetermineRobot();

const inline Robot activeRobot = DetermineRobot();

template <typename T>
T Choose(T compBotValue, T practiceBotValue) {
  return activeRobot == COMP ? compBotValue : practiceBotValue;
}

}  // namespace BotVars