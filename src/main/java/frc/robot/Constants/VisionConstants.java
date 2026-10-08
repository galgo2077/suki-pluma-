package frc.robot.Constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

/** Current Limelight localization configuration. */
public final class VisionConstants {
  private VisionConstants() {}

  public static final String LIMELIGHT_HOSTNAME = "limelight";

  /** Limelight's built-in MJPEG stream. It is available at port 5800 on the robot network. */
  public static final String LIMELIGHT_STREAM_URL =
      "http://" + LIMELIGHT_HOSTNAME + ".local:5800/stream.mjpg";

  public static final AprilTagFieldLayout APRILTAG_LAYOUT =
      AprilTagFields.k2026RebuiltWelded.loadAprilTagLayoutField();

  public static final double MAX_SINGLE_TAG_AMBIGUITY =
      0.30; // TODO TUNE: single-tag ambiguity limit
  public static final double MAX_Z_ERROR_METERS = 0.75; // TODO TUNE: accepted Z error
  // TODO MEASURE: configure robot-to-camera translation in m and rotation in rad in LimelightOS.
  public static final double LINEAR_STD_DEV_BASELINE = 0.02; // TODO TUNE: base linear confidence
  public static final double ANGULAR_STD_DEV_BASELINE = 0.06; // TODO TUNE: base angular confidence
}
