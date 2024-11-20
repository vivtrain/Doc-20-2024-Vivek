// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.Arm.*;

// Use this class to contain all choreographed movements
public class CommandFactory {

	// Don't instantiate, just access the functions in a static way
	private CommandFactory() {}

	public static Command raiseArm() {
		return new SequentialCommandGroup(
			new MoveArmToAngle(45, 2)
		);
	}

	public static Command lowerArm() {
		return new SequentialCommandGroup(
			new MoveArmToAngle(10, 2)
		);
	}
}
