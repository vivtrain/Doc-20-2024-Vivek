// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drivetrain;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {

  // Constants
  private static final double kDriveMaxStatorAmps = 5.0; // TODO: raise when safe
  private static final double kDriveMaxSupplyAmps = 0.0; // TODO: raise when safe
  private static final double kAzimuthMaxStatorAmps = 5.0; // TODO: raise when safe
  private static final double kAzimuthMaxSupplyAmps = 0.0; // TODO: raise when safe
  private static final double kSecondsToRampVoltage = 0.01;
  private static final double kDriveGearReduction = 5.51 / 1;
  private static final double kAzimuthGearReduction = 13.37 / 1;
  private static final double kWheelCircumferenceMeters = Math.PI*Units.inchesToMeters(3.87);
  private static final double kMaxAccelerationRps2 = 0.0; // TODO; set reasonable value
  private static final double kMaxJerkRps3 = 0.0; // TODO; set reasonable value

  // Diffrentiate which module is which
  public static enum ModuleCorners { kFrontLeft, kFrontRight, kRearLeft, kRearRight }
  private ModuleCorners m_whichModule;

  // Talons
  private TalonFX m_driveTalon, m_azimuthTalon;

  // Encoders
  private CANcoder m_encoder;

  // Configs
  private TalonFXConfigurator m_driveConfigurator, m_azimuthConfigurator;
  private CANcoderConfigurator m_encoderConfigurator;

  // Status Signals
  private StatusSignal<Double> m_driveVelocityRpsSignal;
  private StatusSignal<Double> m_drivePositionMetersSignal;
  private StatusSignal<Double> m_azimuthPositionRotationsSignal;

  // Control Requests
  private PositionVoltage m_azmiuthPositionRotationsRequest
    = new PositionVoltage(0.0)
      .withEnableFOC(true)
      .withOverrideBrakeDurNeutral(true)
      .withSlot(0);
  private MotionMagicVelocityVoltage m_driveVelocityRpsRequest
    = new MotionMagicVelocityVoltage(0.0)
      .withEnableFOC(true)
      .withOverrideBrakeDurNeutral(true)
      .withSlot(0);

  // Don't let anyone but Swerve instantiate a module
  protected SwerveModule(
    ModuleCorners whichModule,
      int driveID,
      int azimuthID,
      int encoderID,
      InvertedValue driveDirection,
      InvertedValue azimuthDirection,
      SensorDirectionValue encoderDirection,
      double encoderOffsetRotations) {
    // Initialize
    m_driveTalon = new TalonFX(driveID, "*");
    m_azimuthTalon = new TalonFX(azimuthID, "*");
    m_encoder = new CANcoder(encoderID, "*");
    m_whichModule = whichModule;

    m_driveConfigurator = m_driveTalon.getConfigurator();
    m_azimuthConfigurator = m_azimuthTalon.getConfigurator();
    m_encoderConfigurator = m_encoder.getConfigurator();

    m_driveVelocityRpsSignal = m_driveTalon.getVelocity();
    m_drivePositionMetersSignal = m_driveTalon.getPosition();
    m_azimuthPositionRotationsSignal = m_encoder.getPosition();

		// Limit the stator current for each motor
		CurrentLimitsConfigs driveCurrentLimits = new CurrentLimitsConfigs()
			.withStatorCurrentLimit(kDriveMaxStatorAmps)
			.withStatorCurrentLimitEnable(true)
			.withSupplyCurrentLimit(kDriveMaxSupplyAmps)
			.withSupplyCurrentLimitEnable(false); // TODO: enable
    m_driveConfigurator.apply(driveCurrentLimits);
		CurrentLimitsConfigs azimuthCurrentLimits = new CurrentLimitsConfigs()
			.withStatorCurrentLimit(kAzimuthMaxStatorAmps)
			.withSupplyCurrentLimitEnable(true)
			.withSupplyCurrentLimit(kAzimuthMaxSupplyAmps)
			.withSupplyCurrentLimitEnable(false); // TODO: enable
    m_azimuthConfigurator.apply(azimuthCurrentLimits);

		// Apply voltage ramp rates
		ClosedLoopRampsConfigs closedLoopVoltageRampRate = new ClosedLoopRampsConfigs()
			.withVoltageClosedLoopRampPeriod(kSecondsToRampVoltage);
		OpenLoopRampsConfigs openLoopVoltageRampRate = new OpenLoopRampsConfigs()
			.withVoltageOpenLoopRampPeriod(kSecondsToRampVoltage);
    m_driveConfigurator.apply(closedLoopVoltageRampRate);
    m_driveConfigurator.apply(openLoopVoltageRampRate);
    m_azimuthConfigurator.apply(closedLoopVoltageRampRate);
    m_azimuthConfigurator.apply(openLoopVoltageRampRate);

    // Configure global motor output parameters
		MotorOutputConfigs driveOutputConfig = new MotorOutputConfigs()
			.withDutyCycleNeutralDeadband(0.01) // <1% power will be ignored
			.withInverted(driveDirection)
			.withNeutralMode(NeutralModeValue.Coast);
    m_driveConfigurator.apply(driveOutputConfig);
		MotorOutputConfigs azimuthOutputConfig = new MotorOutputConfigs()
			.withDutyCycleNeutralDeadband(0.01) // <1% power will be ignored
			.withInverted(azimuthDirection)
			.withNeutralMode(NeutralModeValue.Coast);
    m_azimuthConfigurator.apply(azimuthOutputConfig);

		// Apply offset to and determine direction of encoder
		MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs()
			.withMagnetOffset(encoderOffsetRotations)
			.withSensorDirection(encoderDirection); // when looking at the status LED
		m_encoderConfigurator.apply(magnetConfig);

		// Use the CANCoder position (rotations) as the feedback source
    // for closed-loop control of the azimuth position
    // Use the TalonFX rotor position (rotations) as the feeback source
    // for closed-loop control of the drive velocity
		FeedbackConfigs azimuthFeedbackConfig = new FeedbackConfigs()
			.withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
			.withFeedbackRemoteSensorID(m_encoder.getDeviceID())
			.withRotorToSensorRatio(kAzimuthGearReduction)
			.withSensorToMechanismRatio(1 / 1);
    m_azimuthConfigurator.apply(azimuthFeedbackConfig);
		FeedbackConfigs driveFeedbackConfig = new FeedbackConfigs()
			.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
			.withSensorToMechanismRatio(kDriveGearReduction);
    m_driveConfigurator.apply(driveFeedbackConfig);

    // Set up gains
    // Drive gains are motion profiled
    Slot0Configs driveGains = new Slot0Configs()
			.withKS(0.0)
			.withKV(0.0)
			.withKA(0.0)
			.withKP(0.0)
			.withKI(0.0)
			.withKD(0.0);
    m_driveConfigurator.apply(driveGains);
    // Azimuth gains are just straight PID
    Slot0Configs azimuthGains = new Slot0Configs()
			.withKS(0.0)
			.withKP(0.0)
			.withKI(0.0)
			.withKD(0.0);
    m_azimuthConfigurator.apply(azimuthGains);

    // Set up Motion Magic (R)
		MotionMagicConfigs driveMotionMagicConfig = new MotionMagicConfigs() // TODO: figure out proper values
			.withMotionMagicAcceleration(kMaxAccelerationRps2)
			.withMotionMagicJerk(kMaxJerkRps3);
    m_driveConfigurator.apply(driveMotionMagicConfig);
  }

  private double getDriveVelocityMps() {
    return m_driveVelocityRpsSignal.getValue() * kWheelCircumferenceMeters;
  }

  private Rotation2d getAzimuthPosition() {
    return Rotation2d.fromRotations(m_azimuthPositionRotationsSignal.getValue());
  }

  private double getDrivePositionMeters() {
    return m_drivePositionMetersSignal.getValue() * kWheelCircumferenceMeters;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePositionMeters(), getAzimuthPosition());
  }

  private void requestDriveVelocity(double metersPerSecond) {
    double rps = metersPerSecond / kWheelCircumferenceMeters;
    m_driveTalon.setControl(m_driveVelocityRpsRequest.withVelocity(rps));
  }

  private void requestAzimuthPosition(Rotation2d position) {
    m_azimuthTalon.setControl(
      m_azmiuthPositionRotationsRequest.withPosition(position.getRotations())
    );
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
      getDriveVelocityMps(),
      getAzimuthPosition());
  }

  public void requestState(SwerveModuleState state) {
    // Optimize the requested state (i.e. take the shortest path to the correct azimuth)
    state = SwerveModuleState.optimize(state, getAzimuthPosition());
    // Perform cosine optimization (i.e. scale the speed based on azimuthal error)
    /* See WPILib Docs for more info https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html */
    state.speedMetersPerSecond *= state.angle.minus(getAzimuthPosition()).getCos();
    requestDriveVelocity(state.speedMetersPerSecond);
    requestAzimuthPosition(state.angle);
  }

  public void stop() {
    m_driveTalon.stopMotor();
    m_azimuthTalon.stopMotor();
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
      m_driveVelocityRpsSignal,
      m_drivePositionMetersSignal,
      m_azimuthPositionRotationsSignal);
    outputTelemetry();
  }

  private void outputTelemetry() {
    SmartDashboard.putNumber(
      "Swerve/" + m_whichModule.toString() + "/Drive Mps",
      getDriveVelocityMps());
    SmartDashboard.putNumber(
      "Swerve/" + m_whichModule.toString() + "/Drive Meters",
      getDrivePositionMeters());
    SmartDashboard.putNumber(
      "Swerve/" + m_whichModule.toString() + "/Azimuth Degrees",
      getAzimuthPosition().getDegrees());
  }
}
