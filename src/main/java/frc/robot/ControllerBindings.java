// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.io.Controllers.RegisteredController;
import frc.robot.Commands.Arm.*;
import frc.robot.Commands.Intake.RunIntake;
import frc.robot.Commands.CommandFactory;

// Use this class to map Commands to controllers
public class ControllerBindings {
  private RegisteredController m_baseDriver = new RegisteredController(0);
  private RegisteredController m_coDriver = new RegisteredController(1);

  // Singleton ensures only one object exists
  private static ControllerBindings m_instance;
  public static ControllerBindings getInstance() {
    if (m_instance == null)
      m_instance = new ControllerBindings();
    return m_instance;
  }

  // Do not try to construct an object directly
  private ControllerBindings() {}

  // Map all buttons and triggers in the constructor
  // Instantiate in Robot class
  public void bindCommandsToControllers() {

    // Base driver mappings
    m_baseDriver.registerButtonMap(XboxController.Button.kA.value)
      .onTrue(new SequentialCommandGroup()); // empty command group for testing

    // Co-driver mappings
    m_coDriver.registerTriggerMap(XboxController.Axis.kRightY.value, 0.25)
      .whileTrue(new MoveArmDutyCycle(m_coDriver, XboxController.Axis.kRightY.value, false));
    m_coDriver.registerButtonMap(XboxController.Button.kY.value)
      .onTrue(CommandFactory.raiseArm());
    m_coDriver.registerButtonMap(XboxController.Button.kA.value)
      .onTrue(CommandFactory.lowerArm());
    m_coDriver.registerButtonMap(XboxController.Button.kRightBumper.value)
      .whileTrue(new RunIntake(0.5, true));
    m_coDriver.registerTriggerMap(XboxController.Axis.kRightTrigger.value, 0.5)
      .onTrue(CommandFactory.fire());
  }
}
