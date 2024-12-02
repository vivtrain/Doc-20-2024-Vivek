// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;

public class RevToRPM extends Command {

  private Shooter m_shooter = Shooter.getInstance();
  private double m_rpm, m_tolerance;
  private boolean m_holdRPM;

  /** Rev the shooter up to a particular RPM
   * @param rpm angular speed of flywheel
   * @param tolerance how close is close enough
   * @param holdRPM set to true if the shooter should continue revving after it reaches the setpoint */
  public RevToRPM(double rpm, double tolerance, boolean holdRPM) {
    addRequirements();
    m_rpm = rpm;
    m_tolerance = tolerance;
    m_holdRPM = holdRPM;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.revToRPM(m_rpm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!m_holdRPM)
      m_shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return MathUtil.isNear(m_rpm, m_shooter.getRPM(), m_tolerance);
  }
}
