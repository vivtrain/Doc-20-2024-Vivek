// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.Arm.*;
import frc.robot.Commands.Intake.*;
import frc.robot.Commands.Shooter.*;

// Use this class to contain all choreographed movements
public class CommandFactory {

	// Don't instantiate, just access the functions in a static way
	private CommandFactory() {}

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
    return new RunIntake(0.5, true);
  }

  public static Command outtake() {
    return new RunIntake(-0.25, false);
  }

  public static Command stopIntake() {
    return new RunIntake(0, false);
  }

  public static Command fire() {
    return new SequentialCommandGroup(
      new RevToRPM(4000, 100, true),
      new WaitCommand(1.0)
        .deadlineWith(new RunIntake(0.75, false)),
      stopShooter()
    );
  }
}
