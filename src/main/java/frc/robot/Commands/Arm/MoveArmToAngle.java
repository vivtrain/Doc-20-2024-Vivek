// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Arm;

public class MoveArmToAngle extends Command {

  private Arm m_arm = Arm.getInstance();
  private double m_setpoint, m_tolerance;

  /** Move the arm to a specified angle
   * @param degrees desired position of the arm, upwards positive, zero when the aluminum tube is level
   * @param tolerance +/- difference that is considered close enough */
  public MoveArmToAngle(double degrees, double tolerance) {
    addRequirements(m_arm);
    m_setpoint = degrees;
    m_tolerance = tolerance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.moveTo(m_setpoint);
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
    return MathUtil.isNear(m_setpoint, m_arm.getArmAngleDegrees(), m_tolerance);
  }
}
