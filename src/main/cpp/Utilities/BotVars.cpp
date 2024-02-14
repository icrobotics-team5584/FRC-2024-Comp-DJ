#include "utilities/BotVars.h"

Robot BotVars::DetermineRobot() {
  std::string filePath = "/sys/class/net/eth0/address";
  if (std::filesystem::exists(filePath)) {
    std::ifstream file(filePath);
    std::string macAddress;
    file >> macAddress;
    frc::SmartDashboard::PutString("MAC address", macAddress);
    if (macAddress == COMP_BOT_MAC_ADDRESS) {
      frc::SmartDashboard::PutString("active robot", "COMP");
      return Robot::COMP;
    } else if (macAddress == PRACTICE_BOT_MAC_ADDRESS) {
      frc::SmartDashboard::PutString("active robot", "PRACTICE");
      return Robot::PRACTICE;
    }
  }
  frc::SmartDashboard::PutString("active robot",
                                 "ERROR! Could not match MAC address. Defaulting to COMP Bot.");
  return Robot::COMP;
}

template <typename T>
T BotVars::Choose(T compBotValue, T practiceBotValue) {
  return activeRobot == COMP ? compBotValue : practiceBotValue;
}