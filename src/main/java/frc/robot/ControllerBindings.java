// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Controllers.RegisteredController;
import frc.lib.Utility.Utility;
import frc.robot.Commands.CommandBuilder;
import frc.robot.Commands.Arm.*;
import frc.robot.Commands.Shooter.*;
import frc.robot.Commands.Intake.*;
import frc.robot.Commands.Swerve.*;
import frc.robot.Subsystems.Drivetrain.Swerve;

import java.util.function.DoubleSupplier;

// Use this class to map Commands to controllers
public class ControllerBindings {
  private RegisteredController m_baseDriver;
  private RegisteredController m_coDriver;

  // Singleton ensures only one object exists
  private static ControllerBindings m_instance;
  public static ControllerBindings getInstance() {
    if (m_instance == null)
      m_instance = new ControllerBindings();
    return m_instance;
  }

  // Do not try to construct an object directly
  private ControllerBindings() {}

  // Set up all our bindings
  public void bindCommandsToControllers() {
    // bindBaseController();
    bindCoDriverController();
  }

  @SuppressWarnings("unused")
  private void bindBaseController() {
    // Instantiate the controller
    m_baseDriver = new RegisteredController(0);
    // Register mappings and design triggers for when the teleop drive Command should run
    final int leftX = XboxController.Axis.kLeftX.value;
    final int leftY = XboxController.Axis.kLeftY.value;
    final int rightX = XboxController.Axis.kRightX.value;
    final double stickThreshold = 0.1;
    final double translationDownscale = 0.5;
    final double rotationDownscale = 0.1;
    Trigger baseLeftStick = m_baseDriver.register2DAxisMap(leftX, leftY, stickThreshold);
    Trigger baseRightStick = m_baseDriver.registerAxisMap(rightX, stickThreshold);
    // Design mappings from sticks to real world velocities
    /* Note the change in controller axes to field coordinate axes */
    DoubleSupplier leftYStickToXVelocity = () -> {
      // read the controller + rescale the remaining range
      double vx = MathUtil.applyDeadband(-m_baseDriver.getRawAxis(leftY), stickThreshold);
      vx *= Math.abs(vx); // square to produce finer control near zero and preserve direction
      vx *= Swerve.kMaxTranslationSpeedMps; // scale to real world speed
      vx *= translationDownscale;
      return vx;
    };
    DoubleSupplier leftXStickToYVelocity = () -> {
      // read the controller + rescale the remaining range
      double vy = MathUtil.applyDeadband(-m_baseDriver.getRawAxis(leftX), stickThreshold);
      vy *= Math.abs(vy); // square to produce finer control near zero and preserve direction
      vy *= Swerve.kMaxTranslationSpeedMps; // scale to real world speed
      vy *= translationDownscale;
      return vy;
    };
    DoubleSupplier rightXStickToWVelocity = () -> {
      // read the controller + rescale the remaining range
      double vw = MathUtil.applyDeadband(-m_baseDriver.getRawAxis(rightX), stickThreshold);
      vw *= Math.abs(vw); // square to produce finer control near zero and preserve direction
      vw *= Swerve.kMaxRotationalSpeedRadPerSecond; // scale to real world angular speed
      vw *= rotationDownscale;
      return vw;
    };
    // Use our triggers to drive with requested velocities whenever we control with the sticks
    baseLeftStick.or(baseRightStick)
      .whileTrue(
        new Drive(
          leftYStickToXVelocity,
          leftXStickToYVelocity,
          rightXStickToWVelocity));
    
    // Design a function that calculates rotational speeds for aiming
    /*
    DoubleSupplier aimToSpeaker = () -> {
      if (Utility.isOnBlue()) {
        Translation2d blueSpeaker = new Translation2d(0.0, 5.52);
        return Swerve.getInstance().calculateAngularVelocityDemand(blueSpeaker);
      } else if (Utility.isOnRed()) {
        Translation2d redSpeaker = new Translation2d(16.6, 5.52);
        return Swerve.getInstance().calculateAngularVelocityDemand(redSpeaker);
      } else {
        return 0.0;
      }
    };
    m_baseDriver.registerAxisMap(XboxController.Axis.kLeftTrigger.value, 0.5)
      .whileTrue(
        new Drive(
          leftYStickToXVelocity,
          leftXStickToYVelocity,
          aimToSpeaker)); */
  }

  @SuppressWarnings("unused")
  private void bindCoDriverController() {
    // Instantiate the controller
    m_coDriver = new RegisteredController(1);
    // Register all the auxiliary functions of the robot
    m_coDriver.registerAxisMap(XboxController.Axis.kRightY.value, 0.25)
      .whileTrue(new MoveArmDutyCycle(m_coDriver, XboxController.Axis.kRightY.value, true));
    m_coDriver.registerButtonMap(XboxController.Button.kY.value)
      .onTrue(CommandBuilder.raiseArm());
    m_coDriver.registerButtonMap(XboxController.Button.kA.value)
      .onTrue(CommandBuilder.lowerArm());
    m_coDriver.registerButtonMap(XboxController.Button.kRightBumper.value)
      .whileTrue(CommandBuilder.intake());
    m_coDriver.registerButtonMap(XboxController.Button.kLeftBumper.value)
      .whileTrue(CommandBuilder.outtake());
    m_coDriver.registerAxisMap(XboxController.Axis.kRightTrigger.value, 0.5)
      .onTrue(CommandBuilder.fire());
    m_coDriver.registerButtonMap(XboxController.Button.kB.value)
      .onTrue(CommandBuilder.stopShooter());
  }
}
