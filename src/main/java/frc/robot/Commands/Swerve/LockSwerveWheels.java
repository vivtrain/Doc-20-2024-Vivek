// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain.Swerve;

public class LockSwerveWheels extends Command {

  Swerve m_swerve = Swerve.getInstance();

  public LockSwerveWheels() {
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerve.lockWheels();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
