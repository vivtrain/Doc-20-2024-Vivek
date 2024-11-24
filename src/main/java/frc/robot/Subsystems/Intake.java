// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class Intake extends SubsystemBase {

	// Constants
	private static final double kMaxStatorAmps = 40.0;
	private static final double kMaxSupplyAmps = 0.0;
	private static final double kSecondsToRampVoltage = 0.1;

	// Talons
	private TalonFX m_talon = new TalonFX(Ports.CANDevices.Talons.INTAKE, "*");

	// Configurators
	private TalonFXConfigurator m_talonConfigurator = m_talon.getConfigurator();

	// Status Signals
	// (none needed, we can just run this guy straight up)

	// Control Requests
	private DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0)
		.withEnableFOC(true);

	// Breakbeam Sensor
	DigitalInput m_breakBeam = new DigitalInput(Ports.DigitalDevices.INTAKE_BANNER);

	// Singleton pattern ensures only one instance exists in program
	private static Intake m_instance;
	public static Intake getInstance() {
		if (m_instance == null)
			m_instance = new Intake();
		return m_instance;
	}

	private Intake() {
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
			.withInverted(InvertedValue.CounterClockwise_Positive) // Define positive duty cycle => intake
			.withNeutralMode(NeutralModeValue.Coast);
		m_talonConfigurator.apply(motorOutputConfig);

		// Apply voltage ramp rates
		OpenLoopRampsConfigs openLoopVoltageRampRate = new OpenLoopRampsConfigs()
			.withVoltageOpenLoopRampPeriod(kSecondsToRampVoltage);
		m_talonConfigurator.apply(openLoopVoltageRampRate);
	}

	public boolean isBeamBroken() {
		return !m_breakBeam.get();
	}

	public void runWithDutyCycle(double dutyCycle) {
		m_talon.setControl(m_dutyCycleRequest.withOutput(dutyCycle));
	}

  public void stop() {
	m_talon.stopMotor();
  }

	@Override
	public void periodic() {
		outputTelemetry();
	}

	private void outputTelemetry() {
		SmartDashboard.putBoolean("Intake/break beam", isBeamBroken());
	}
}
