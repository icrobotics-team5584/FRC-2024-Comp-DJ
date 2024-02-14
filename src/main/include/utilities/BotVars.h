#pragma once

#include <filesystem>
#include <fstream>
#include <frc/smartdashboard/smartdashboard.h>

namespace BotVars {

enum Robot { COMP, PRACTICE };
static constexpr inline auto COMP_BOT_MAC_ADDRESS = "00:15:5d:45:d7:5f";
static constexpr inline auto PRACTICE_BOT_MAC_ADDRESS = "ff:ff:ff:ff:ff:ff";

Robot DetermineRobot();

static inline const Robot activeRobot = DetermineRobot();

template <typename T>
T Choose(T compBotValue, T practiceBotValue);

}  // namespace BotVars