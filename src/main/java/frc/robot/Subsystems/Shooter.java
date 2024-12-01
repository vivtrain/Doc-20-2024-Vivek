// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class Shooter extends SubsystemBase {

	// Constants
	private static final double kMaxStatorAmps = 40.0;
	private static final double kMaxSupplyAmps = 0.0;
	private static final double kSecondsToRampVoltage = 0.1;
  private static final double kVelocityRpsToVoltageFeedForward = 12.0 / (5800 / 60) * 1.04; // Volt per RPS

  // Talons
  private TalonFX m_talon = new TalonFX(Ports.CANDevices.Talons.SHOOTER, "*");

  // Configurators
  private TalonFXConfigurator m_talonConfigurator = m_talon.getConfigurator();

  // Status Signals
  private StatusSignal<Double> m_velocityRpsSignal = m_talon.getVelocity();

  // Control Requests
  private VelocityVoltage m_RpsToVoltageRequest = new VelocityVoltage(0)
  	.withEnableFOC(true)
    .withSlot(0);

  private static Shooter m_instance;
  public static Shooter getInstance() {
    if (m_instance == null)
      m_instance = new Shooter();
    return m_instance;
  } 

  private Shooter() {
		// Limit the stator current for each motor
		CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
			.withStatorCurrentLimit(kMaxStatorAmps)
			.withStatorCurrentLimitEnable(true)
			.withSupplyCurrentLimit(kMaxSupplyAmps)
			.withSupplyCurrentLimitEnable(false); // TODO: enable
    m_talonConfigurator.apply(currentLimits);

		// Configure global motor output parameters
		MotorOutputConfigs motorOutputConfig = new MotorOutputConfigs()
			.withDutyCycleNeutralDeadband(0.01) // <1% power will be ignored
			.withInverted(InvertedValue.Clockwise_Positive)
			.withNeutralMode(NeutralModeValue.Coast);
		m_talonConfigurator.apply(motorOutputConfig);

		// Apply voltage ramp rates
		OpenLoopRampsConfigs openLoopVoltageRampRate = new OpenLoopRampsConfigs()
			.withVoltageOpenLoopRampPeriod(kSecondsToRampVoltage);
		m_talonConfigurator.apply(openLoopVoltageRampRate);

    // Set up feeback (this is redundant since thse are the defaults)
    FeedbackConfigs feedbackConfig = new FeedbackConfigs()
      .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
		m_talonConfigurator.apply(feedbackConfig);

    // Set up gains
    Slot0Configs gains = new Slot0Configs()
      .withKV(kVelocityRpsToVoltageFeedForward) // TODO: determine proper gains
      .withKP(0.5)
      .withKI(0.0)
      .withKD(0.0);
		m_talonConfigurator.apply(gains);
  }

  /** Get the angular speed of the shooter wheels
   * @return angular speed in RPM */
  public double getRPM() {
    return m_velocityRpsSignal.getValue() * 60;
  }

  /** Get the current setpoint of the shooter
   * @return setpoint in RPM */
  public double getSetpointRPM() {
    return m_talon.getClosedLoopReference().getValue() * 60;
  }

  /** Set a goal shooter speed
   * @param rpm goal speed */
  public void revToRPM(double rpm) {
    m_talon.setControl(m_RpsToVoltageRequest.withVelocity(rpm / 60));
  }

  /** Stop the shooter */
  public void stop() {
    m_talon.stopMotor();
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(m_velocityRpsSignal);
		outputTelemetry();
  }

  private void outputTelemetry() {
    SmartDashboard.putNumber("Shooter/talon setpoint RPM", getSetpointRPM());
    SmartDashboard.putNumber("Shooter/real RPM", getRPM());
  }
}
