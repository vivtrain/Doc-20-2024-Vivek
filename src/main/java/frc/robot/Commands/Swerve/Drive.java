// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain.Swerve;
import java.util.function.DoubleSupplier;

public class Drive extends Command {

  Swerve m_swerve = Swerve.getInstance();
  DoubleSupplier m_vx, m_vy, m_vw;

  /** Drive the robot at the requested linear and angular velocity in field coordinates
   * @param vx function that returns meters/second, positive facing away from your alliance wall
   * @param vy function that returns meters/second, positive facing left field boundary
   * @param vw function that returns radians/second, CCW+, with zero aligned with +x */
  public Drive(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier vw) {
    addRequirements(m_swerve);
    m_vx = vx;
    m_vy = vy;
    m_vw = vw;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerve.requestChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
      m_vx.getAsDouble(),
      m_vy.getAsDouble(),
      m_vw.getAsDouble(),
      m_swerve.getGyroAngularPosition()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
