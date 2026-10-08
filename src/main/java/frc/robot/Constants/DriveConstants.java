package frc.robot.Constants;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Subsystems.drive.Motor.module.SteeringReferenceMode;
import java.util.OptionalDouble;
import java.util.OptionalInt;

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

  // Fill these from verified robot measurements before enabling path following.
  public static final OptionalDouble ROBOT_MASS_KG = OptionalDouble.empty();
  public static final OptionalDouble ROBOT_MOI_KG_METERS_SQUARED = OptionalDouble.empty();
  public static final OptionalDouble WHEEL_RADIUS_METERS = OptionalDouble.empty();
  public static final OptionalDouble DRIVE_GEAR_RATIO = OptionalDouble.empty();
  public static final OptionalDouble WHEEL_COF = OptionalDouble.empty();
  public static final OptionalDouble DRIVE_CURRENT_LIMIT_AMPS = OptionalDouble.empty();
  public static final OptionalInt DRIVE_MOTORS_PER_MODULE = OptionalInt.empty();

  // SIMULATION TEST ONLY: not verified hardware measurements.
  public static final double SIM_TEST_ROBOT_MASS_KG = 50.0;
  public static final double SIM_TEST_ROBOT_MOI_KG_METERS_SQUARED = 5.0;
  public static final double SIM_TEST_WHEEL_RADIUS_METERS = 0.0508;
  public static final double SIM_TEST_DRIVE_GEAR_RATIO = 6.0;
  public static final double SIM_TEST_WHEEL_COF = 1.0;
  public static final double SIM_TEST_DRIVE_CURRENT_LIMIT_AMPS = 40.0;
  public static final int SIM_TEST_DRIVE_MOTORS_PER_MODULE = 1;

  public static final SteeringReferenceMode[] STEERING_REFERENCE_MODES = {
    SteeringReferenceMode.ABSOLUTE_ENCODER, // TODO CALIBRATE: FL absolute encoder offset
    SteeringReferenceMode.ABSOLUTE_ENCODER, // TODO CALIBRATE: FR absolute encoder offset
    SteeringReferenceMode.ABSOLUTE_ENCODER, // TODO CALIBRATE: BL absolute encoder offset
    // TODO CALIBRATE / TEMPORARY FALLBACK: physically align BR to its zero reference before
    // startup.
    SteeringReferenceMode.MANUAL_ZERO
  };

  /** One physical module in fixed FL, FR, BL, BR order. */
  public record ModuleHardware(
      Integer driveCanId,
      Integer turnCanId,
      Integer canCoderCanId,
      Integer throughBoreDio,
      Boolean driveInverted,
      Boolean turnInverted,
      Integer driveCurrentLimitAmps,
      Integer turnCurrentLimitAmps,
      Double absoluteOffsetRadians) {}

  // TODO HARDWARE: enter IDs, inversion, current limits, and measured absolute offsets.
  // FL, FR, and BL use CANcoders; BR uses its Thrifty Through Bore Encoder on DIO.
  public static final ModuleHardware[] MODULE_HARDWARE = {
    new ModuleHardware(null, null, null, null, null, null, null, null, null),
    new ModuleHardware(null, null, null, null, null, null, null, null, null),
    new ModuleHardware(null, null, null, null, null, null, null, null, null),
    new ModuleHardware(null, null, null, null, null, null, null, null, null)
  };

  // TODO HARDWARE: Pigeon CAN ID, mounting, and yaw sign.
  public static final Integer PIGEON_CAN_ID = null;

  // TODO MEASURE: wheel circumference and both module reductions, expressed as encoder units.
  public static final double DRIVE_POSITION_METERS_PER_MOTOR_ROTATION = Double.NaN;
  public static final double DRIVE_VELOCITY_METERS_PER_SECOND_PER_RPM = Double.NaN;
  public static final double TURN_POSITION_RADIANS_PER_MOTOR_ROTATION = Double.NaN;
  public static final double TURN_VELOCITY_RADIANS_PER_SECOND_PER_RPM = Double.NaN;

  // TODO TUNE: SPARK MAX drive/turn closed-loop gains.
  public static final double DRIVE_KP = Double.NaN;
  public static final double TURN_KP = Double.NaN;

  public static boolean realHardwareConfigurationValid() {
    if (PIGEON_CAN_ID == null || MODULE_HARDWARE.length != 4) return false;
    for (int i = 0; i < MODULE_HARDWARE.length; i++) {
      ModuleHardware module = MODULE_HARDWARE[i];
      boolean hasExpectedAbsoluteEncoder =
          i == 3 ? module.throughBoreDio() != null : module.canCoderCanId() != null;
      if (module.driveCanId() == null
          || module.turnCanId() == null
          || !hasExpectedAbsoluteEncoder
          || module.driveInverted() == null
          || module.turnInverted() == null) return false;
    }
    return true;
  }
}
