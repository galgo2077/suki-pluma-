package frc.robot.Constants;

import edu.wpi.first.math.geometry.Translation2d;

/** Shooter configuration. Automatic feeding stays disabled until commissioning is complete. */
public final class ShooterConstants {
  private ShooterConstants() {}

  // TODO HARDWARE: assign Kraken #1 CAN ID.
  public static final Integer KRAKEN_1_CAN_ID = null;
  // TODO HARDWARE: assign Kraken #2 CAN ID.
  public static final Integer KRAKEN_2_CAN_ID = null;
  // TODO HARDWARE: assign NEO feeder CAN ID.
  public static final Integer FEEDER_CAN_ID = null;
  // TODO HARDWARE: verify each motor's positive direction.
  public static final Boolean KRAKEN_1_INVERTED = null;
  public static final Boolean KRAKEN_2_INVERTED = null;
  public static final Boolean FEEDER_INVERTED = null;
  // TODO MEASURE: motor rotations per flywheel rotation.
  public static final double MOTOR_ROTATIONS_PER_FLYWHEEL_ROTATION = Double.NaN;
  // TODO CALIBRATE: acceptable flywheel RPM tolerance.
  public static final double RPM_TOLERANCE = Double.NaN;
  // TODO CALIBRATE: Phoenix 6 velocity-loop gains for both Kraken motors.
  public static final Double KRAKEN_KP = null;
  public static final Double KRAKEN_KI = null;
  public static final Double KRAKEN_KD = null;
  public static final Double KRAKEN_KS = null;
  public static final Double KRAKEN_KV = null;
  // TODO CALIBRATE: NEO feeder output while automatic feeding is enabled.
  public static final double REAL_FEEDER_OUTPUT = Double.NaN;
  // TODO MEASURE: shooter exit position relative to the robot center, in robot coordinates.
  public static final Translation2d SHOOTER_EXIT_FROM_ROBOT_CENTER_METERS = null;

  // SIMULATION ONLY: these values are not hardware calibration values.
  public static final double SIM_TEST_TARGET_RPM = 3000.0;
  public static final double SIM_RPM_TOLERANCE = 100.0;
  public static final double SIM_FLYWHEEL_TIME_CONSTANT_SECONDS = 0.25;
  public static final double SIM_FEEDER_OUTPUT = 0.5;

  /** Required controller identity values. */
  public static boolean realHardwareConfigurationValid() {
    return KRAKEN_1_CAN_ID != null
        && KRAKEN_2_CAN_ID != null
        && FEEDER_CAN_ID != null
        && KRAKEN_1_INVERTED != null
        && KRAKEN_2_INVERTED != null
        && FEEDER_INVERTED != null;
  }
}
