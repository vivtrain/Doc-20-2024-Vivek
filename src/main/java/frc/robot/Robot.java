// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.CommandBuilder;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Drivetrain.Swerve;

public class Robot extends TimedRobot {

  // Initailze subsystems by constructing them here
  Arm m_arm = Arm.getInstance();
  Intake m_intake = Intake.getInstance();
  Shooter m_shooter = Shooter.getInstance();
  Swerve m_swerve = Swerve.getInstance();

  @Override
  public void robotInit() {
    // This is a good place to establish any default commands using Subsystem.setDefaultCommand
    m_swerve.setDefaultCommand(CommandBuilder.lockSwerveWheels());
    // Set up Path Planner (do this after subsystem inits, but before creating other commands)
    PathPlannerAutonomous.configure();
    PathPlannerAutonomous.registerNamedCommands();
    // Configure teleop button bindings
    ControllerBindings.getInstance().bindCommandsToControllers();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    m_arm.stop();
    m_intake.stop();
    m_shooter.stop();
    m_swerve.stop();
  }

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
