// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drivetrain;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Limelight.Limelight;
import frc.lib.Limelight.LimelightHelpers.PoseEstimate;
import frc.robot.Ports;

public class Swerve extends SubsystemBase {

  // Constants
  private static final SwerveModuleState[] kLockedStates = {
    new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
    new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
    new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
    new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
  };
  public static final double kTrackWidthMeters = Units.inchesToMeters(23.25); // distance from left wheel to right wheel
  public static final double kWheelBaseMeters = Units.inchesToMeters(23.25); // distance from front wheel to back wheel
  public static final double kRadius = Math.hypot(kWheelBaseMeters/2, kTrackWidthMeters/2); // distance from center to a wheel
  public static final double kMaxTranslationSpeedMps = Units.feetToMeters(20.36);
  public static final double kMaxRotationalSpeedRadPerSecond = kMaxTranslationSpeedMps / kRadius; // omega = v/r
  public static final double kMaxTranslationAccelerationMps2 = Units.feetToMeters(30); // TODO: check
  public static final double kMaxRotationalAccelerationRadPerSecond2 = kMaxTranslationAccelerationMps2 / kRadius; // alpha = a/r

  // Consturct all the modules
  /* Modules zeroed along robot coordinate system:
   * - Forward drive direction facing +x (i.e. forward)
   * - Bevels facing +y (i.e. left)
   * - Azimuth direction being CCW+ as viewed from the above the robot */
  SwerveModule m_frontLeftModule = new SwerveModule(
    SwerveModule.ModuleCorners.kFrontLeft,
    Ports.CANDevices.Talons.SWERVE_FRONT_LEFT_DRIVE,
    Ports.CANDevices.Talons.SWERVE_FRONT_LEFT_AZIMUTH,
    Ports.CANDevices.Encoders.SWERVE_FRONT_LEFT,
    InvertedValue.Clockwise_Positive, // TODO: determine empirically
    InvertedValue.Clockwise_Positive, // TODO: determine empirically
    SensorDirectionValue.CounterClockwise_Positive,
    Rotation2d.fromDegrees(161.2)
  );
  SwerveModule m_frontRightModule = new SwerveModule(
    SwerveModule.ModuleCorners.kFrontRight,
    Ports.CANDevices.Talons.SWERVE_FRONT_RIGHT_DRIVE,
    Ports.CANDevices.Talons.SWERVE_FRONT_RIGHT_AZIMUTH,
    Ports.CANDevices.Encoders.SWERVE_FRONT_RIGHT,
    InvertedValue.Clockwise_Positive, // TODO: determine empirically
    InvertedValue.Clockwise_Positive, // TODO: determine empirically
    SensorDirectionValue.CounterClockwise_Positive,
    Rotation2d.fromDegrees(-169.2)
  );
  SwerveModule m_rearLeftModule = new SwerveModule(
    SwerveModule.ModuleCorners.kRearLeft,
    Ports.CANDevices.Talons.SWERVE_REAR_LEFT_DRIVE,
    Ports.CANDevices.Talons.SWERVE_REAR_LEFT_AZIMUTH,
    Ports.CANDevices.Encoders.SWERVE_REAR_LEFT,
    InvertedValue.Clockwise_Positive, // TODO: determine empirically
    InvertedValue.Clockwise_Positive, // TODO: determine empirically
    SensorDirectionValue.CounterClockwise_Positive,
    Rotation2d.fromDegrees(-38.1)
  );
  SwerveModule m_rearRightModule = new SwerveModule(
    SwerveModule.ModuleCorners.kRearRight,
    Ports.CANDevices.Talons.SWERVE_REAR_RIGHT_DRIVE,
    Ports.CANDevices.Talons.SWERVE_REAR_RIGHT_AZIMUTH,
    Ports.CANDevices.Encoders.SWERVE_REAR_RIGHT,
    InvertedValue.Clockwise_Positive, // TODO: determine empirically
    InvertedValue.Clockwise_Positive, // TODO: determine empirically
    SensorDirectionValue.CounterClockwise_Positive,
    Rotation2d.fromDegrees(113.3)
  );

  // Collect into an array
  /* EVERYTHING STAYS IN THIS ORDER */
  private SwerveModule[] m_modules = {
    m_frontLeftModule,
    m_frontRightModule,
    m_rearLeftModule,
    m_rearRightModule
  };

  // Kinetmatics (defined in robot coordinates)
  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    new Translation2d(kTrackWidthMeters/2, kWheelBaseMeters/2),  // front left  => +x, +y
    new Translation2d(kTrackWidthMeters/2, -kWheelBaseMeters/2), // front right => +x, -y
    new Translation2d(-kTrackWidthMeters/2, kWheelBaseMeters/2), // rear left   => -x, +y
    new Translation2d(-kTrackWidthMeters/2, -kWheelBaseMeters/2) // rear right  => -x, -y
  );

  // Pose Estimator
  SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
    m_kinematics,
    Rotation2d.fromDegrees(0.0),
    getModulePositions(),
    new Pose2d(),
    VecBuilder.fill(0.05, 0.05, 0.1), // TODO: tune, trust this more maybe?
    VecBuilder.fill(0.1, 0.1, 0.2)
  );

  // Pigeon + Config + Status Signals
  private Pigeon2 m_gyro = new Pigeon2(Ports.CANDevices.PIGEON, "*");
  private Pigeon2Configurator m_gyroConfigurator = m_gyro.getConfigurator();
  private StatusSignal<Double> m_angularPositionDegreesSignal = m_gyro.getYaw();
  private StatusSignal<Double> m_angularVelocityDpsSignal = m_gyro.getAngularVelocityZWorld();
  /* Note: Pigeon2.getAngle() and Pigeon2.getRate() follow NED axis conventions
   *    which require CW+ orientations. Since we want to keep all of our coordinate systems
   *    for drive consistently CCW+, we will use the raw status signals, which are not
   *    negated. See the translation at https://api.ctr-electronics.com/phoenix6/release/java/src-html/com/ctre/phoenix6/hardware/Pigeon2.html#line.234 */

  // Limelight
  Limelight m_limelight = new Limelight("limelight");

  // Motion Profiled Controller for Turning
  ProfiledPIDController m_turningController = new ProfiledPIDController(
    5.0,
    0.0,
    0.0,
    new Constraints(kMaxRotationalSpeedRadPerSecond, kMaxRotationalAccelerationRadPerSecond2)
  );

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

    // Zero position on init
    resetPosition(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
  }

  /** Reset the robot's position manually. Resets the gyro as well.
   * @param fieldCoords robot's position in field coordinates, i.e.
   *   <ul>
   *     <li> +x is facing away from your alliance station in meters </li>
   *     <li> +y is facing left in meters </li>
   *     <li> rotation is CCW+ in radians </li>
   *   </ul> */
  public void resetPosition(Pose2d fieldCoords) {
    // Reset the gyro, not really necessary but helpful if it matches the pose estimator
    m_gyro.setYaw(fieldCoords.getRotation().getDegrees());
    // Overwrite the pose estimator's output position
    m_poseEstimator.resetPosition(
      getGyroAngularPosition(),
      getModulePositions(),
      fieldCoords);
  }

  /** Drive the robot relative to its internal coordinate system
   *  @param chassisSpeeds robot-relative ChassisSpeeds coordinates are defined as:
   *    <ul>
   *      <li> +vx => forward (from robot's perspective) in meters/sec </li>
   *      <li> +vy => left (from robot's perspective) in meters/sec </li>
   *      <li> +vw => counter-clockwise in radians/sec  </li>
   *    </ul>
   *  @apiNote See WPILib Docs for more info: https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/intro-and-chassis-speeds.html */
  public void requestChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    // Determine necessary module states from the requested robot linear and angular velocities
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
    // Renormalize all module speeds if any is above the attainable max
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, kMaxTranslationSpeedMps);
    // Request the respective module state from each module
    for (int m = 0; m < m_modules.length; m++)
      m_modules[m].requestState(moduleStates[m]);
  }

  /** Calculates the necessary rotational speed for aiming to a particular location.
   * @param pointToTurnTo (x,y) point on the field (blue-side origin)
   * @return angular velocity in radians per second */
  public double calculateAngularVelocityDemand(Translation2d pointToTurnTo) {
    Pose2d currentPose = getEstimatedPose();
    Translation2d currentLocation = currentPose.getTranslation();
    double currentOrientation = MathUtil.angleModulus(currentPose.getRotation().getRadians());
    Translation2d difference = currentLocation.minus(pointToTurnTo);
    double setpoint = MathUtil.angleModulus(difference.getAngle().getRadians());
    return m_turningController.calculate(currentOrientation, setpoint);
  }

  /** Lock the swerve wheeels in an X pattern */
  public void lockWheels() {
    for (int m = 0; m < m_modules.length; m++)
      m_modules[m].requestState(kLockedStates[m]);
  }

  /** Stop all the drive and azimuth motors wherever they are */
  public void stop() {
    for (SwerveModule module : m_modules)
      module.stop();
  }

  /** Get the robot's angular position from wherever the gyro is zeroed
   * @return Rotation2d representing the gyro reading (CCW+) */
  public Rotation2d getGyroAngularPosition() {
    return Rotation2d.fromDegrees(m_angularPositionDegreesSignal.getValue());
  }

  /** Get the robot's angular velocity
   * @return Rotation2d representing the gyro's velocity reading (CCW+) */
  public Rotation2d getGyroAngularVelocity() {
    return Rotation2d.fromDegrees(m_angularVelocityDpsSignal.getValue());
  }

  /** Get the current robot chassis speeds in its own internal coordinates
   *  @return robot-relative ChassisSpeeds, defined as:
   *    <ul>
   *      <li> +vx => forward (from robot's perspective) in meters/sec </li>
   *      <li> +vy => left (from robot's perspective) in meters/sec </li>
   *      <li> +vw => counter-clockwise in radians/sec  </li>
   *    </ul>
   *  @apiNote See WPILib Docs for more info: https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/intro-and-chassis-speeds.html */
  public ChassisSpeeds getChassisSpeedsRobotRelative() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[m_modules.length];
    for (int m = 0; m < m_modules.length; m++)
      moduleStates[m] = m_modules[m].getState();
    ChassisSpeeds state = m_kinematics.toChassisSpeeds(moduleStates);
    return state;
  }

  /** Get the current robot speed along field-relative coordinates
   *  @return field-relative ChassisSpeeds, defined as:
   *  <ul>
   *    <li> +vx => forward (from blue alliance driver station) in meters/sec </li>
   *    <li> +vy => left (from blue alliance driver station) in meters/sec </li>
   *    <li> +vw => increasing counter-clockwise in radians/sec </li>
   *  </ul>
   * @apiNote See WPILib Docs for more info: https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/intro-and-chassis-speeds.html */
  public ChassisSpeeds getChassisSpeedsFieldRelative() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
      getChassisSpeedsRobotRelative(),
      getGyroAngularPosition());
  }

  /** Get the estimated location and orientation of the robot, including vision, gyro, and
   *  odometry measurements.
   * @return estimated position */
  public Pose2d getEstimatedPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(m_angularPositionDegreesSignal, m_angularVelocityDpsSignal);
    // WPILib Pose Estimator Recommendations: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html
    updatePosewithOdometry();
    updatePoseWithLimelight();
    outputTelemetry();
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[m_modules.length];
    for (int m = 0; m < m_modules.length; m++)
      modulePositions[m] = m_modules[m].getPosition();
    return modulePositions;
  }

  private void updatePosewithOdometry() {
    m_poseEstimator.updateWithTime(
      Timer.getFPGATimestamp(),
      getGyroAngularPosition(),
      getModulePositions());
  }

  private void updatePoseWithLimelight() {
    Optional<PoseEstimate> limelightBotPose = m_limelight.getBotPoseEstimate();
    if (limelightBotPose.isPresent()) {
      PoseEstimate estimate = limelightBotPose.get();
      // Distrust tags more when they are far away
      double distrust = 1.0 * estimate.avgTagDist; // TODO: tune
      Matrix<N3,N1> visionStdDevs;
      // Trust measurements more when they include multiple tags
      double xyScale = (estimate.tagCount > 1) ? 0.08 : 0.1; // TODO: tune
      double orientationScale = Double.POSITIVE_INFINITY; // ignore this measurement entirely
      visionStdDevs = VecBuilder.fill(xyScale, xyScale, orientationScale).times(distrust);
      m_poseEstimator.addVisionMeasurement(
        estimate.pose,
        Timer.getFPGATimestamp() - m_limelight.getTotalLatencySeconds(), // compensate for latency
        visionStdDevs);
    }
  }

  private void outputTelemetry() {
    // Gyro
    SmartDashboard.putNumber("Swerve/Gyro/Position (deg)",
      getGyroAngularPosition().getDegrees());
    SmartDashboard.putNumber("Swerve/Gyro/Velocity (dps)",
      getGyroAngularVelocity().getDegrees());
    ChassisSpeeds chassisSpeeds = getChassisSpeedsRobotRelative();
    // Odometry (robot coords)
    SmartDashboard.putNumber("Swerve/Odometry/X Velocity (mps)",
      chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Swerve/Odometry/Y Velocity (mps)",
      chassisSpeeds.vyMetersPerSecond);
    double translationVelocity
      = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Swerve/Odometry/Translation Velocity (mps)",
      translationVelocity);
    SmartDashboard.putNumber("Swerve/Odometry/Angular Velocity (rad per s)",
      chassisSpeeds.omegaRadiansPerSecond);
    // Pose Estimator (field coords)
    Pose2d estimatedPose = getEstimatedPose();
    SmartDashboard.putNumber("Swerve/Pose Estimator/X Position (m)", estimatedPose.getX());
    SmartDashboard.putNumber("Swerve/Pose Estimator/Y Position (m)", estimatedPose.getY());
    SmartDashboard.putNumber("Swerve/Pose Estimator/Orienation (deg)",
      estimatedPose.getRotation().getDegrees());
  }
}
