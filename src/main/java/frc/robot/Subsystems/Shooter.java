// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class Shooter extends SubsystemBase {

	// Constants
	private static final double kMaxStatorAmps = 5.0;
	private static final double kMaxSupplyAmps = 0.0;
	private static final double kSecondsToRampVoltage = 0.1;

  // Talons
  private TalonFX m_talon = new TalonFX(Ports.CANDevices.Talons.SHOOTER, "*");

  // Configurators
  private TalonFXConfigurator m_talonConfigurator = m_talon.getConfigurator();

  // Status Signals
  private StatusSignal<Double> m_velocityRpsSignal = m_talon.getVelocity();

  // Control Requests
  private MotionMagicVelocityVoltage m_RpsRequest = new MotionMagicVelocityVoltage(0)
  	.withEnableFOC(true)
    .withSlot(0);

  // Managed Subsystem State
  private Double m_setpointRPM = null;

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
			.withInverted(InvertedValue.Clockwise_Positive) // TODO: check
			.withNeutralMode(NeutralModeValue.Coast);
		m_talonConfigurator.apply(motorOutputConfig);

		// Apply voltage ramp rates
		OpenLoopRampsConfigs openLoopVoltageRampRate = new OpenLoopRampsConfigs()
			.withVoltageOpenLoopRampPeriod(kSecondsToRampVoltage);
		m_talonConfigurator.apply(openLoopVoltageRampRate);

    // Set up feeback (this is redundant since thse are the defaults)
    FeedbackConfigs feedbackConfig = new FeedbackConfigs()
      .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
      .withRotorToSensorRatio(1/1);
		m_talonConfigurator.apply(feedbackConfig);

    // Set up Motion Magic (R)
    MotionMagicConfigs motionMagicConfig = new MotionMagicConfigs()
      .withMotionMagicExpo_kV(0.0); // TODO: figure out what makes sense here
		m_talonConfigurator.apply(motionMagicConfig);

    // Set up gains
    Slot0Configs gains = new Slot0Configs()
      .withKV(0.13) // TODO: determine proper gains
      .withKP(0.0)
      .withKI(0.0)
      .withKD(0.0);
		m_talonConfigurator.apply(gains);
  }

  public double getRPM() {
    return m_velocityRpsSignal.getValue() * 60;
  }

  public void revToRPM(double rpm) {
    m_setpointRPM = rpm;
    m_talon.setControl(m_RpsRequest.withVelocity(rpm / 60));
  }

  public void stop() {
    m_setpointRPM = 0.0;
    m_talon.stopMotor();
  }

  @Override
  public void periodic() {
    m_velocityRpsSignal.refresh();
		outputTelemetry();
  }

  private void outputTelemetry() {
    SmartDashboard.putNumber("Shooter/setpoint RPM", m_setpointRPM);
    SmartDashboard.putNumber("Shooter/real RPM", getRPM());
  }
}
