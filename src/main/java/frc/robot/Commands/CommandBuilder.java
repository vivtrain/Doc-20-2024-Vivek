// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.Utility.Utility;
import frc.robot.Commands.Arm.*;
import frc.robot.Commands.Intake.*;
import frc.robot.Commands.Shooter.*;
import frc.robot.Commands.Swerve.*;
import frc.robot.Subsystems.Drivetrain.Swerve;

// Use this class to contain all choreographed movements
public class CommandBuilder {

	// Don't instantiate, just access the functions in a static way
	private CommandBuilder() {}

	public static Command raiseArm() {
		return new MoveArmToAngle(45, 2);
	}

	public static Command lowerArm() {
		return new MoveArmToAngle(30, 2);
	}

  public static Command stopShooter() {
    return new RevToRPM(0, 10e3, false);
  }

  public static Command intake() {
    return new RunIntake(0.75, true);
  }

  public static Command outtake() {
    return new RunIntake(-0.25, false);
  }

  public static Command stopIntake() {
    return new RunIntake(0, false);
  }

  public static Command lockSwerveWheels() {
    return new InstantCommand(() -> Swerve.getInstance().lockWheels(), Swerve.getInstance());
  }

  public static Command fire() {
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        raiseArm(),
        new RevToRPM(3000, 100, true)),
      new WaitCommand(1.0)
        .deadlineWith(new RunIntake(0.75, false)),
      stopShooter()
    );
  }

  public static Command resetSwervePosition(Pose2d fieldCoords) {
    return new InstantCommand(() -> Swerve.getInstance().resetPosition(fieldCoords));
  }
}
