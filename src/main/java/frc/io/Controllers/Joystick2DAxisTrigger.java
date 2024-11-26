// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.io.Controllers;

import edu.wpi.first.util.ErrorMessages;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Joystick2DAxisTrigger extends Trigger {

  /** Fires when a the norm of a pair of axes exceeds the given threshold
   * @param genericHID the human interface device to check 
   * @param xAxis the horizontal axis to read, assumed to read zero with no human input
   * @param yAxis the vertical axis to read, assumed to read zero with no human input
   * @param threshold the +/- threshold outside of which should cause the trigger to fire
   */
  public Joystick2DAxisTrigger(GenericHID genericHID, int xAxis, int yAxis, double threshold) {
    super(
      () -> Math.hypot(genericHID.getRawAxis(xAxis), genericHID.getRawAxis(yAxis)) > threshold);
    ErrorMessages.requireNonNullParam(
      genericHID,
      genericHID.getName(),
      this.getClass().getSimpleName() + "constructor");
  }

}
