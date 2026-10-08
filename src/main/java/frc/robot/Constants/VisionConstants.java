package frc.robot.Constants;

import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;

/** Current Limelight localization configuration. */
public final class VisionConstants {
  private VisionConstants() {}

  public static final String LIMELIGHT_HOSTNAME = "limelight";
  public static final Field APRILTAG_LAYOUT =
      Fields.FRC_2026_REBUILT_WELDED.loadField(); // FIXED SPEC: official 2026 REBUILT field layout

  public static final double MAX_SINGLE_TAG_AMBIGUITY =
      0.30; // TODO TUNE: single-tag ambiguity limit
  public static final double MAX_Z_ERROR_METERS = 0.75; // TODO TUNE: accepted Z error
  public static final double MAX_MEGATAG2_ANGULAR_VELOCITY_RAD_PER_SEC =
      Math.toRadians(720.0); // TODO TUNE: MegaTag2 angular-rate rejection limit

  // TODO MEASURE: configure measured robot-space Limelight transform in LimelightOS.
  public static final double LINEAR_STD_DEV_BASELINE = 0.02; // TODO TUNE: base linear confidence
  public static final double ANGULAR_STD_DEV_BASELINE = 0.06; // TODO TUNE: base angular confidence
  public static final double MEGATAG2_LINEAR_STD_DEV_FACTOR =
      0.5; // TODO TUNE: increase to trust MegaTag2 less
  public static final double MEGATAG2_ANGULAR_STD_DEV_FACTOR =
      Double.POSITIVE_INFINITY; // TODO TUNE: MegaTag2 angular confidence
}
