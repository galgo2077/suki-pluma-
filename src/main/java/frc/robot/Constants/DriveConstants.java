package frc.robot.Constants;

import frc.robot.subsystems.drive.Motor.module.SteeringReferenceMode;
import org.wpilib.math.geometry.Translation2d;

/** Simulation-only values. TODO Phase 2: replace with measured Suki-Pluma values. */
public final class DriveConstants {
  private DriveConstants() {}

  public static final double MAX_SPEED = 4.5;
  public static final double MAX_OMEGA = Math.PI;
  // TODO Phase 2: tune heading PID on real Suki-Pluma.
  public static final double HEADING_KP = 4.0;
  public static final double HEADING_KI = 0.0;
  public static final double HEADING_KD = 0.0;
  public static final double TRACK_WIDTH = 0.55; // TODO Phase 2: measured value
  public static final double WHEEL_BASE = 0.55; // TODO Phase 2: measured value
  public static final Translation2d[] MODULE_TRANSLATIONS = {
    new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // FL
    new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // FR
    new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // BL
    new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2) // BR
  };

  /** Temporary fallback: FL/FR/BL use CANcoder; BR is physically aligned then manually zeroed. */
  public static final SteeringReferenceMode[] STEERING_REFERENCE_MODES = {
    SteeringReferenceMode.ABSOLUTE_ENCODER,
    SteeringReferenceMode.ABSOLUTE_ENCODER,
    SteeringReferenceMode.ABSOLUTE_ENCODER,
    SteeringReferenceMode.MANUAL_ZERO
  };
}
