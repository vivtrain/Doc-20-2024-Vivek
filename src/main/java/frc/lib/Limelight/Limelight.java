// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.Limelight;

import java.util.Optional;

import frc.lib.Limelight.LimelightHelpers.*;
import frc.lib.Utility.Utility;

public class Limelight {

  private String m_name;

  public Limelight(String name) {
    m_name = name;
  }

  /** Checks if Limelight sees valid targets (e.g. AprilTags, game pieces, etc.).
   *  Calling this with a parameter of 1 is equivalent to checking "tv"
   * @param atLeastThisMany minimum amount of targets seen to return true
   * @return whether Limelight sees that many targets or more */
  public boolean seesValidTargets(int atLeastThisMany) {
    return LimelightHelpers.getTargetCount(m_name) > atLeastThisMany;
  }

  /** Get the raw results for a specific AprilTag if it is seen.
   * @param tagID the tag to look for
   * @return either the tag object or null */
  public Optional<RawFiducial> getAprilTagInfo(int tagID) {
    RawFiducial[] results = LimelightHelpers.getRawFiducials(m_name);
    for (RawFiducial result : results)
      if (result.id == tagID)
        return Optional.of(result);
    return null;
  }

  /** Get the the target area as a fraction (in the range [0-1]) of the total image area
   *  if the target is seen.
   * @return either target area or null */
  public Optional<Double> getTargetArea() {
    if (seesValidTargets(1))
      return Optional.of(LimelightHelpers.getTA(m_name));
    else
      return null;
  }

  /** Get the angle (right positive, center zero) to a specific tag if it is seen
   * @param tagID the tag to look for
   * @return either the angle in degrees or null */
  public Optional<Double> getAngleToTagDegrees(int tagID) {
    Optional<RawFiducial> tag = getAprilTagInfo(tagID);
    if (tag == null)
      return null;
    return Optional.of(tag.get().txnc);
  }

  /** Get the total latency of the Limelight data including capture and pipeline. This does
   *  not include any other sources of latency like NetworkTables.
   * @return the total computed latency in seconds */
  public double getTotalLatencySeconds() {
    return (LimelightHelpers.getLatency_Capture(m_name)
        + LimelightHelpers.getLatency_Pipeline(m_name)) / 1000.0;
  }

  /** Switch Limelight pipelines.
   * @param pipeline index of pipeline ranging in [0,9]
   * @throws IndexOutOfBoundsException if passed an invalid pipeline index */
  public void setPipeline(int pipeline) throws IndexOutOfBoundsException {
    if (pipeline >= 0 && pipeline <= 9)
      LimelightHelpers.setPipelineIndex(m_name, pipeline);
    else
      throw new IndexOutOfBoundsException(
        "Limelight only has pipelines in the range 0-9 (inclusive), was passed "
        + Integer.toString(pipeline));
  }

  /** Get a pose estimate using MegaTag2, Optional since DS Alliance may not be set.
   * @return estimated pose or null */
  public Optional<PoseEstimate> getBotPoseEstimate() {
    if (Utility.isOnBlue())
      return Optional.of(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_name));
    else if (Utility.isOnRed())
      return Optional.of(LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(m_name));
    else
      return null;
  }
}
