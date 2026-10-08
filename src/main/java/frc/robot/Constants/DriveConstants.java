package frc.robot.Constants;

import edu.wpi.first.math.geometry.Translation2d;

/** Current drivetrain values awaiting real-robot verification. */
public final class DriveConstants {
  private DriveConstants() {}

  public static final double TRACK_WIDTH = 0.55; // TODO MEASURE: track width
  public static final double WHEEL_BASE = 0.55; // TODO MEASURE: wheelbase
  public static final Translation2d[] MODULE_TRANSLATIONS = {
    new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // TODO MEASURE: FL module position
    new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // TODO MEASURE: FR module position
    new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // TODO MEASURE: BL module position
    new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2) // TODO MEASURE: BR module position
  };
}
