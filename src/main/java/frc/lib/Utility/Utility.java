// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.Utility;

import java.util.Optional;
import edu.wpi.first.wpilibj.DriverStation;

/** This class contains static convenience functions */
public class Utility {

  private Utility() {
    throw new AssertionError("Do not instantiate Utility class");
  }

  public static boolean isOnRed() {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent())
      return alliance.get() == DriverStation.Alliance.Red;
    else
      return false;
  }

  public static boolean isOnBlue() {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent())
      return alliance.get() == DriverStation.Alliance.Blue;
    else
      return false;
  }
}
