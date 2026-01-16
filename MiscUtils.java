// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

/**
 * Misc utility functions
 */
public final class MiscUtils {

  public static double GetAngleDifferenceClockwise_double(double from, double to) 
  {
    double diff = (from - to) % 360.0;  // diff now in (-360.0 ... 360.0) range
    if (diff >= 180.0) {
      diff -= 360.0;                    // diff now in (-360.0 ... 180.0) range
    } else if (diff < -180.0) {
      diff += 360.0;                    // diff now in [-180.0 ... 180.0) range
    }
    return diff;
  }

  // Given an axis reading from -1 to 1, scale it to 0..1 for the safety factor.
  // Safety Scale (-1 to 1 --> 0.0 to 1)
  @SuppressWarnings("unused")
  private double scaleStickToSafetyFactor(double rawStick) {
    return (rawStick + 1.0) / 2.0;
  }


}
