// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Ports;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;

public class Arm extends SubsystemBase {

	// Constants
	private static final double kEncoderOffsetRotations = -0.196; // rotations of magnet, TODO: determine empirically
	private static final double kGearReduction = 240 / 1;
	private static final double kMaxStatorAmps = 40.0; // TODO: raise once safety established
	private static final double kMaxSupplyAmps = 0.0; // TODO: raise once safety established
	private static final double kSecondsToRampVoltage = 0.1;
	private static final double kForwardLimitMotorRotations = 1.20; // TODO: determine empirically
	private static final double kReverseLimitMotorRotations = 1.00; // TODO: determine empirically
	private static final int kAscentGains = 0;
	private static final double kMaxVelocityRps = 5800 / 60 / kGearReduction * 1.0; // TODO: check for accuracy
	private static final double kMaxAccelerationRps2 = kMaxVelocityRps * 3;
	private static final double kMaxJerkRps3 = kMaxAccelerationRps2 * 10;

	// Talons
	private TalonFX m_armTalon = new TalonFX(Ports.CANDevices.Talons.ARM_LEADER, "*");
	private TalonFX m_armTalonFollower = new TalonFX(Ports.CANDevices.Talons.ARM_FOLLOWER, "*");

	// Encoders
	private CANcoder m_encoder = new CANcoder(Ports.CANDevices.Encoders.ARM, "*");

	// Configurators
	private TalonFXConfigurator m_armConfiguratorLeader = m_armTalon.getConfigurator();
	private TalonFXConfigurator m_armTalonConfiguratorFollower = m_armTalon.getConfigurator();
	private CANcoderConfigurator m_encoderConfigurator = m_encoder.getConfigurator();

	// Status Signals
	private StatusSignal<Double> m_encoderPositionSignal = m_encoder.getAbsolutePosition();
	private StatusSignal<Double> m_armTalonPositionSignal = m_armTalon.getPosition();

	// Control Requests
	private MotionMagicVoltage m_positionControlReqeust = new MotionMagicVoltage(0.0)
		.withEnableFOC(true)
		.withOverrideBrakeDurNeutral(true);
	private DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0.0)
		.withEnableFOC(true);

	// Managed Subsystem State
	private Double m_setpointDegrees = null;

	// Singleton pattern ensures only one instance exists in program
	private static Arm m_instance;
	public static Arm getInstance() {
		if (m_instance == null)
			m_instance = new Arm();
		return m_instance;
	}

	private Arm() {
		// Limit the stator current for each motor
		CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
			.withStatorCurrentLimit(kMaxStatorAmps)
			.withStatorCurrentLimitEnable(true)
			.withSupplyCurrentLimit(kMaxSupplyAmps)
			.withSupplyCurrentLimitEnable(false); // TODO: enable
		m_armConfiguratorLeader.apply(currentLimits);
		m_armTalonConfiguratorFollower.apply(currentLimits);

		// Setup actuation limits
		SoftwareLimitSwitchConfigs actuationLimits = new SoftwareLimitSwitchConfigs()
			.withForwardSoftLimitThreshold(kForwardLimitMotorRotations)
			.withForwardSoftLimitEnable(false)
			.withReverseSoftLimitThreshold(kReverseLimitMotorRotations)
			.withReverseSoftLimitEnable(false);
		m_armConfiguratorLeader.apply(actuationLimits);

		// Configure global motor output parameters
		MotorOutputConfigs motorOutputConfig = new MotorOutputConfigs()
			.withDutyCycleNeutralDeadband(0.01) // <1% power will be ignored
			.withInverted(InvertedValue.Clockwise_Positive) // ccw positive facing the rotor end
			.withNeutralMode(NeutralModeValue.Coast);
		m_armConfiguratorLeader.apply(motorOutputConfig);

		// Apply offset to and determine direction of encoder
		MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs()
			.withMagnetOffset(kEncoderOffsetRotations)
			.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf) // we want to read negative encoder positions
			.withSensorDirection(SensorDirectionValue.Clockwise_Positive); // when looking at the status LED
		m_encoderConfigurator.apply(magnetConfig);

		// Apply voltage ramp rates
		ClosedLoopRampsConfigs closedLoopVoltageRampRate = new ClosedLoopRampsConfigs()
			.withVoltageClosedLoopRampPeriod(kSecondsToRampVoltage);
		m_armConfiguratorLeader.apply(closedLoopVoltageRampRate);
		OpenLoopRampsConfigs openLoopVoltageRampRate = new OpenLoopRampsConfigs()
			.withVoltageOpenLoopRampPeriod(kSecondsToRampVoltage);
		m_armConfiguratorLeader.apply(openLoopVoltageRampRate);

		// Use the CANCoder position (rotations) as the feedback source for closed-loop control
		FeedbackConfigs feedbackConfig = new FeedbackConfigs()
			.withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
			.withFeedbackRemoteSensorID(m_encoder.getDeviceID())
			.withRotorToSensorRatio(kGearReduction)
			.withSensorToMechanismRatio(1 / 1)
      .withFeedbackRotorOffset(0.0);
		m_armConfiguratorLeader.apply(feedbackConfig);
		// Set up gain profiles
		Slot0Configs ascentConfig = new Slot0Configs() // TODO: figure out proper values
			.withGravityType(GravityTypeValue.Arm_Cosine) // 0 should be horizontal (i.e. max holding torque)
			.withKG(0.04 * 12.0)
			.withKS(0.0)
			.withKV(12.0 / kMaxVelocityRps)
			//.withKV(0.0)
			//.withKA(0.02)
			.withKP(12.0 / (30.0/360.0))
			.withKI(0.0)
			.withKD(0.0);
		m_armConfiguratorLeader.apply(ascentConfig);

		// Set up Motion Magic (R)
		MotionMagicConfigs motionMagicConfig = new MotionMagicConfigs() // TODO: figure out proper values
			.withMotionMagicCruiseVelocity(kMaxVelocityRps)
			.withMotionMagicAcceleration(kMaxAccelerationRps2)
			.withMotionMagicJerk(kMaxJerkRps3);
		m_armConfiguratorLeader.apply(motionMagicConfig);

		// Make the follower follow the leader, but in the opposite direction
		Follower followCtrlRequest = new Follower(m_armTalon.getDeviceID(), true);
		m_armTalonFollower.setControl(followCtrlRequest);
	}

  /** Get the arm angle in degrees, upwards positive, zero where the aluminum tube is level
   * @return arm angle */
	public double getArmAngleDegrees() {
		// Encoder will read Arm rotations so translate to degrees
		double degrees = m_encoderPositionSignal.getValue() * 360;
		return degrees;
	}

  /** Set a goal state in degrees for the arm, upwards positive, zero where the aluminum tube is level
   * @param degrees goal state of the arm */
	public void moveTo(double degrees) {
		// Expects Arm degrees but Phoenix will use Arm rotations as feedback
		int whichSlot = kAscentGains;//(degrees > getArmAngleDegrees()) ? kAscentGains : kDescentGains;
		m_setpointDegrees = degrees;
		m_armTalon.setControl(m_positionControlReqeust
			.withSlot(whichSlot)
			.withPosition(degrees / 360)
		);
	}

  /** Hold the arm's most recent setpoint position */
	public void hold() {
		if (m_setpointDegrees == null)
			return; // skip the rest, since there's no setpoint currently
		m_armTalon.setControl(m_positionControlReqeust
			.withSlot(kAscentGains)
			.withPosition(m_setpointDegrees / 360)
		);
	}

  // TODO: debug, remove this functionality
	public void moveManually(double dutyCycle) {
		m_setpointDegrees = null;
		m_armTalon.setControl(m_dutyCycleRequest
			.withOutput(dutyCycle)
		);
	}

  /** Stop the arm. This should stop both motors if follwer is configured correctly. */
	public void stop() {
		m_armTalon.stopMotor();
	}

	@Override
	public void periodic() {
		// Refresh all status signals once per robot loop
		BaseStatusSignal.refreshAll(m_armTalonPositionSignal, m_encoderPositionSignal);
		// Hold your position (if there exists a setpoint)
		//hold();
		// Place all readouts in below function
		outputTelemetry();

		if (getArmAngleDegrees() > 80 || getArmAngleDegrees() < -1) {
			stop();
		}
	}

	private void outputTelemetry() {
		SmartDashboard.putNumber("Arm/encoder position (deg)", getArmAngleDegrees());
		SmartDashboard.putNumber("Arm/talon position (arm rot)", m_armTalonPositionSignal.getValue());
		SmartDashboard.putNumber("Arm/duty cycle", m_armTalon.getDutyCycle().refresh().getValue());
    SmartDashboard.putNumber("Arm/PID setpoint", m_armTalon.getClosedLoopReference().refresh().getValue());
    SmartDashboard.putNumber("Arm/PID error", m_armTalon.getClosedLoopError().refresh().getValue());
    SmartDashboard.putNumber("Arm/PID output", m_armTalon.getClosedLoopOutput().refresh().getValue());
	}
}
