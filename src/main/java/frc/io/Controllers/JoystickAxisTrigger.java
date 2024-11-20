// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.io.Controllers;

import edu.wpi.first.util.ErrorMessages;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class JoystickAxisTrigger extends Trigger {

  public JoystickAxisTrigger(GenericHID genericHID, int axis, double threshold) {
    super(() -> Math.abs(genericHID.getRawAxis(axis)) > threshold);
    ErrorMessages.requireNonNullParam(genericHID, genericHID.getName(), "JoystickAxisTrigger constructor");
  }

}
