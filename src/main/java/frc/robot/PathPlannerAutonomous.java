// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.Utility.Utility;
import frc.robot.Commands.CommandBuilder;
import frc.robot.Subsystems.Drivetrain.Swerve;

public class PathPlannerAutonomous {

  private static final PIDConstants kTranslationGains
    = new PIDConstants(0, 0, 0); // TODO: tune
  private static final PIDConstants kRotationGains 
    = new PIDConstants(0, 0, 0); // TODO: tune

  // Treat this like a utility class i.e. don't instantiate, just call configure
  private PathPlannerAutonomous() {}

  public static void configure() {
    Swerve m_swerve = Swerve.getInstance();
    HolonomicPathFollowerConfig holonomicPathFollowerConfig
      = new HolonomicPathFollowerConfig(
        kTranslationGains,
        kRotationGains,
        Swerve.kMaxTranslationSpeedMps,
        Swerve.kRadius,
        null);
    AutoBuilder.configureHolonomic(
      () -> m_swerve.getEstimatedPose(),
      (Pose2d pose) -> m_swerve.resetPosition(pose),
      () -> m_swerve.getChassisSpeedsRobotRelative(),
      (ChassisSpeeds chassisSpeeds) -> m_swerve.requestChassisSpeeds(chassisSpeeds),
      holonomicPathFollowerConfig,
      () -> Utility.isOnRed(),
      m_swerve);
  }

  public static void registerNamedCommands() {
    NamedCommands.registerCommand("stop", CommandBuilder.lockSwerveWheels());
    // TODO: whatever else is needed for auto
  }
}
