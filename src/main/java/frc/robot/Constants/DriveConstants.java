package frc.robot.Constants;

import frc.robot.Subsystems.drive.Motor.module.SteeringReferenceMode;
import org.wpilib.math.geometry.Translation2d;

/** Current drivetrain values awaiting real-robot verification. */
public final class DriveConstants {
  private DriveConstants() {}

  public static final double MAX_SPEED = 4.5; // TODO TUNE: real maximum linear and module speed
  public static final double MAX_OMEGA = Math.PI; // TODO TUNE: real maximum angular speed
  public static final double HEADING_KP = 4.0; // TODO TUNE: real heading kP
  public static final double HEADING_KI = 0.0; // TODO TUNE: real heading kI
  public static final double HEADING_KD = 0.0; // TODO TUNE: real heading kD
  public static final double TRACK_WIDTH = 0.55; // TODO MEASURE: track width
  public static final double WHEEL_BASE = 0.55; // TODO MEASURE: wheelbase
  public static final Translation2d[] MODULE_TRANSLATIONS = {
    new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // TODO MEASURE: FL module position
    new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // TODO MEASURE: FR module position
    new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // TODO MEASURE: BL module position
    new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2) // TODO MEASURE: BR module position
  };

  public static final SteeringReferenceMode[] STEERING_REFERENCE_MODES = {
    SteeringReferenceMode.ABSOLUTE_ENCODER, // TODO CALIBRATE: FL absolute encoder offset
    SteeringReferenceMode.ABSOLUTE_ENCODER, // TODO CALIBRATE: FR absolute encoder offset
    SteeringReferenceMode.ABSOLUTE_ENCODER, // TODO CALIBRATE: BL absolute encoder offset
    // TODO CALIBRATE / TEMPORARY FALLBACK: physically align BR to its zero reference before
    // startup.
    SteeringReferenceMode.MANUAL_ZERO
  };
}
