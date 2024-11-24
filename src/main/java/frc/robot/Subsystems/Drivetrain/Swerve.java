// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class Swerve extends SubsystemBase {

  // Constants
  private static final SwerveModuleState[] kLockedStates = {
    new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
    new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
    new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
    new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
  };
  private static final double kPhysicalMaxSpeedMps = Units.feetToMeters(19); // TODO: check

  // Consturct all the modules
  SwerveModule m_frontLeftModule = new SwerveModule(
    SwerveModule.ModuleCorners.kFrontLeft,
    Ports.CANDevices.Talons.SWERVE_FRONT_LEFT_DRIVE,
    Ports.CANDevices.Talons.SWERVE_FRONT_LEFT_AZIMUTH,
    Ports.CANDevices.Encoders.SWERVE_FRONT_LEFT,
    InvertedValue.Clockwise_Positive, // TODO: determine empirically
    InvertedValue.Clockwise_Positive, // TODO: determine empirically
    SensorDirectionValue.Clockwise_Positive, // TODO: determine empirically
    0 // TODO: determine empirically
  );
  SwerveModule m_frontRightModule = new SwerveModule(
    SwerveModule.ModuleCorners.kFrontRight,
    Ports.CANDevices.Talons.SWERVE_FRONT_RIGHT_DRIVE,
    Ports.CANDevices.Talons.SWERVE_FRONT_RIGHT_AZIMUTH,
    Ports.CANDevices.Encoders.SWERVE_FRONT_RIGHT,
    InvertedValue.Clockwise_Positive, // TODO: determine empirically
    InvertedValue.Clockwise_Positive, // TODO: determine empirically
    SensorDirectionValue.Clockwise_Positive, // TODO: determine empirically
    0 // TODO: determine empirically
  );
  SwerveModule m_rearLeftModule = new SwerveModule(
    SwerveModule.ModuleCorners.kRearLeft,
    Ports.CANDevices.Talons.SWERVE_REAR_LEFT_DRIVE,
    Ports.CANDevices.Talons.SWERVE_REAR_LEFT_AZIMUTH,
    Ports.CANDevices.Encoders.SWERVE_REAR_LEFT,
    InvertedValue.Clockwise_Positive, // TODO: determine empirically
    InvertedValue.Clockwise_Positive, // TODO: determine empirically
    SensorDirectionValue.Clockwise_Positive, // TODO: determine empirically
    0 // TODO: determine empirically
  );
  SwerveModule m_rearRightModule = new SwerveModule(
    SwerveModule.ModuleCorners.kRearRight,
    Ports.CANDevices.Talons.SWERVE_REAR_RIGHT_DRIVE,
    Ports.CANDevices.Talons.SWERVE_REAR_RIGHT_AZIMUTH,
    Ports.CANDevices.Encoders.SWERVE_REAR_RIGHT,
    InvertedValue.Clockwise_Positive, // TODO: determine empirically
    InvertedValue.Clockwise_Positive, // TODO: determine empirically
    SensorDirectionValue.Clockwise_Positive, // TODO: determine empirically
    0 // TODO: determine empirically
  );

  // Collect into an array
  private SwerveModule[] m_modules = {
    m_frontLeftModule,
    m_frontRightModule,
    m_rearLeftModule,
    m_rearRightModule
  };

  // Kinetmatics
  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics();
  SwerveModuleState[] m_moduleStates = new SwerveModuleState[m_modules.length];
  ChassisSpeeds m_chassisState = new ChassisSpeeds();

  // Pigeon
  private Pigeon2 m_gyro = new Pigeon2(Ports.CANDevices.PIGEON, "*");

  // Config
  private Pigeon2Configurator m_gyroConfigurator = m_gyro.getConfigurator();

  // Status Signals
  /*
   * Note: Pigeon2.getAngle() and Pigeon2.getRate() follow NED axis conventions
   *    which require CW+ orientations. Since we want to keep all of our coordinate systems
   *    for drive consistently CCW+, we will use the raw status signals, which are not
   *    negated. See the translation at https://api.ctr-electronics.com/phoenix6/release/java/src-html/com/ctre/phoenix6/hardware/Pigeon2.html#line.234
   */
  private StatusSignal<Double> m_angularPositionDegreesSignal = m_gyro.getYaw();
  private StatusSignal<Double> m_angularVelocityDpsSignal = m_gyro.getAngularVelocityZWorld();

  // Singleton code patter ensures only one object exists
  private static Swerve m_instance = null;
  public static Swerve getInstance() {
    if (m_instance == null)
      m_instance = new Swerve();
    return m_instance;
  }

  // Do not construct directly, use getInstance
  private Swerve() {
    // Establish the mount orientation of the gyro
    MountPoseConfigs gyroMountPoseConfig = new MountPoseConfigs()
      .withMountPosePitch(0.0)
      .withMountPoseRoll(0.0)
      .withMountPoseYaw(0.0);
    m_gyroConfigurator.apply(gyroMountPoseConfig);

    // Zero gyro when the object is first constructed
    resetGyroOrientation();
  }

  // Zero the gyro
  public void resetGyroOrientation() {
    m_gyro.reset();
  }

  // Drive the robot relative to its own coordinate system
  /* Note: The ChassisSpeeds coordinates are defined as:
   *  +vx => forward
   *  +vy => left
   *  +vw => counter-clockwise in radians
   */
  public void requestChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    m_chassisState = chassisSpeeds;
    // Determine necessary module states from the requested robot linear and angular velocities
    m_moduleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
    // Renormalize all module speeds if any is above the attainable max
    SwerveDriveKinematics.desaturateWheelSpeeds(m_moduleStates, kPhysicalMaxSpeedMps);
    // This should never happen, so crash if it does
    if (m_moduleStates.length != m_modules.length)
      throw new RuntimeException("Tried to assign " + Integer.toString(m_moduleStates.length)
        + " module states to " + Integer.toString(m_modules.length) + " modules!");
    // Request the respective module state from each module
    for (int m = 0; m < m_modules.length; m++)
      m_modules[m].requestModuleState(m_moduleStates[m]);
  }

  // Lock wheels in an X pattern
  public void lockWheels() {
    for (int m = 0; m < m_modules.length; m++)
      m_modules[m].requestModuleState(kLockedStates[m]);
  }

  public void stop() {
    for (SwerveModule module : m_modules)
      module.stop();
  }

  // CCW+ from wherever zeroed
  public Rotation2d getGyroAngularPosition() {
    return Rotation2d.fromDegrees(m_angularPositionDegreesSignal.getValue());
  }

  // CCW+
  public Rotation2d getGyroAngularVelocity() {
    return Rotation2d.fromDegrees(m_angularVelocityDpsSignal.getValue());
  }

  /* Note: The ChassisSpeeds coordinates are defined as:
   *  +vx => forward
   *  +vy => left
   *  +vw => increasing counter-clockwise in radians
   */
  public ChassisSpeeds getChassisRelativeSpeeds() {
    for (int m = 0; m < m_modules.length; m++)
      m_moduleStates[m] = m_modules[m].getState();
    ChassisSpeeds state = m_kinematics.toChassisSpeeds(m_moduleStates);
    state.omegaRadiansPerSecond = getGyroAngularVelocity().getRadians();
    return state;
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(m_angularPositionDegreesSignal, m_angularVelocityDpsSignal);
    outputTelemetry();
  }

  private void outputTelemetry() {
  }
}
