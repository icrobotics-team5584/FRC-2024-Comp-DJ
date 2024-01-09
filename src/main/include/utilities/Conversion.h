// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/velocity.h>
#include <units/length.h>
#include <frc/geometry/Rotation2d.h>
#include <numbers>

namespace Conversions {
constexpr int FALCON_TICS_PER_REVOLUTION = 2048;
constexpr double TAU = std::numbers::pi * 2;

/**
 * Converts falcon velocity units (falocn encoder tics per 100 milliseconds) of
 * falcon motors driving a drivebase to real world robot speed (meters per sec).
 * 
 * @param falconVelocity falcon encoder vel measurement in tics / 100 ms
 * @param gearRatio the drive base gear ratio on the drive motors
 * @param wheelRadius the radius of the drivebase wheels.
 */
units::meters_per_second_t FalconVelToRobotVel(int falconVelocity,
                                               double gearRatio,
                                               units::meter_t wheelRadius);

/**
 * Converts a the distance of a falcon motor (measured in encoder tics) to a 
 * the distance in meters
 * 
 * @param tics angle of the falcon motor in falcon encoder tics
 * @param gearRatio the gear ratio between the falcon and mechanism output.
 * @param wheelRadius the radius of the drivebase wheels.
 */
units::meter_t FalconTicsToMeters(const int tics, double gearRatio, units::meter_t wheelRadius);

/**
 * Converts a robot velocity in meters per second to the angular velocity that
 * the falcon drive motors would need to spin at, measureed in falcon encoder 
 * tics per 100 milliseconds.
 * 
 * @param robotVelocity how fast the robot is travelling / should travel
 * @param gearRatio the drive base gear ratio on the drive motors
 * @param wheelRadius the radius of the drivebase wheels. 
 */
double RobotVelToFalconVel(units::meters_per_second_t robotVelocity,
                           double gearRatio, units::meter_t wheelRadius);

/**
 * Converts a the angle of a falcon motor (measured in encoder tics) to a 
 * the angle of the mechanism it is attached to.
 * 
 * @param tics angle of the falcon motor in falcon encoder tics
 * @param gearRatio the gear ratio between the falcon and mechanism output.
 */
frc::Rotation2d FalconTicsToOutputRotations(const int tics,
                                            const double gearRatio);
}  // namespace Conversions
