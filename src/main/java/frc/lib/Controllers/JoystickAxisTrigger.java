// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.Controllers;

import edu.wpi.first.util.ErrorMessages;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class JoystickAxisTrigger extends Trigger {

  /** Fires when a GenericHID axis moves outside of a threshold centered around zero
   * @param genericHID the human interface device to check
   * @param axis the axis to read, assumed to read zero with no human input
   * @param threshold the +/- threshold outside of which should cause the trigger to fire
   */
  public JoystickAxisTrigger(GenericHID genericHID, int axis, double threshold) {
    super(() -> Math.abs(genericHID.getRawAxis(axis)) > threshold);
    ErrorMessages.requireNonNullParam(
      genericHID,
      genericHID.getName(),
      this.getClass().getSimpleName() + "constructor");
  }

}
