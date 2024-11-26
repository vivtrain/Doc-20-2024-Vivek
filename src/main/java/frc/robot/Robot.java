// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Drivetrain.Swerve;

public class Robot extends TimedRobot {

  @Override
  public void robotInit() {
    // Initailze subsystems by constructing them here
    /* Note: even though this is not strictly necessary, it's best to pre-compute and config
     * as much as possible */
    // This is a good place to establish any default commands using Subsystem.setDefaultCommand
    Arm.getInstance();
    Intake.getInstance();
    Shooter.getInstance();
    Swerve.getInstance();
    // Configure teleop button bindings
    ControllerBindings.getInstance().bindCommandsToControllers();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
