// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;

public class RunIntake extends Command {

  private Intake m_intake = Intake.getInstance();
  private double m_dutyCycle;
  private boolean m_useSensor;

  /** Run the intake
   * @param dutyCycle [-1,1] fraction of supply voltage, positive => intake, negative => outtake
   * @param useSensor set to true if the Command should finish upon beam-break trip */
  public RunIntake(double dutyCycle, boolean useSensor) {
    addRequirements(m_intake);
    m_dutyCycle = dutyCycle;
    m_useSensor = useSensor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.runWithDutyCycle(m_dutyCycle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_useSensor && m_intake.isBeamBroken();
  }
}
