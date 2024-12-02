// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Arm;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Arm;

public class MoveArmDutyCycle extends Command {

  Arm m_arm = Arm.getInstance();
  GenericHID m_hid;
  int m_axis;
  int m_reverse;

  /** Move the arm with input from a controller axis
   * @param hid human-interface device (e.g. Joystick)
   * @param axis ID of axis
   * @param reverseAxis set to true if input needs to be negated */
  public MoveArmDutyCycle(GenericHID hid, int axis, boolean reverseAxis) {
    addRequirements(m_arm);
    m_hid = hid;
    m_axis = axis;
    m_reverse = reverseAxis ? -1 : 1;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double downscale = 0.1;
    m_arm.moveManually(m_hid.getRawAxis(m_axis) * downscale * m_reverse);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_arm.moveTo(m_arm.getArmAngleDegrees()); TODO restore this once close loop control is fixed
    m_arm.moveManually(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
