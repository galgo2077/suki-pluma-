package frc.robot.Constants;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import java.util.Optional;

/** PathPlanner-only configuration derived from the drivetrain's single source of truth. */
public final class PathplannerConstants {
  // TODO TUNE: characterize path-following gains before enabling competition autonomous.
  public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.0);
  public static final PIDConstants ROTATION_PID = new PIDConstants(0.0);

  private PathplannerConstants() {}

  /** Returns empty until every physical value required by PathPlanner has been verified. */
  public static Optional<RobotConfig> robotConfig() {
    if (DriveConstants.ROBOT_MASS_KG.isEmpty()
        || DriveConstants.ROBOT_MOI_KG_METERS_SQUARED.isEmpty()
        || DriveConstants.WHEEL_RADIUS_METERS.isEmpty()
        || DriveConstants.DRIVE_GEAR_RATIO.isEmpty()
        || DriveConstants.WHEEL_COF.isEmpty()
        || DriveConstants.DRIVE_CURRENT_LIMIT_AMPS.isEmpty()
        || DriveConstants.DRIVE_MOTORS_PER_MODULE.isEmpty()) {
      return Optional.empty();
    }

    return Optional.of(
        createRobotConfig(
            DriveConstants.ROBOT_MASS_KG.getAsDouble(),
            DriveConstants.ROBOT_MOI_KG_METERS_SQUARED.getAsDouble(),
            DriveConstants.WHEEL_RADIUS_METERS.getAsDouble(),
            DriveConstants.WHEEL_COF.getAsDouble(),
            DriveConstants.DRIVE_GEAR_RATIO.getAsDouble(),
            DriveConstants.DRIVE_CURRENT_LIMIT_AMPS.getAsDouble(),
            DriveConstants.DRIVE_MOTORS_PER_MODULE.getAsInt()));
  }

  /** Provides a non-hardware RobotConfig only for desktop movement tests. */
  public static RobotConfig simulationRobotConfig() {
    return robotConfig()
        .orElseGet(
            () ->
                createRobotConfig(
                    DriveConstants.SIM_TEST_ROBOT_MASS_KG,
                    DriveConstants.SIM_TEST_ROBOT_MOI_KG_METERS_SQUARED,
                    DriveConstants.SIM_TEST_WHEEL_RADIUS_METERS,
                    DriveConstants.SIM_TEST_WHEEL_COF,
                    DriveConstants.SIM_TEST_DRIVE_GEAR_RATIO,
                    DriveConstants.SIM_TEST_DRIVE_CURRENT_LIMIT_AMPS,
                    DriveConstants.SIM_TEST_DRIVE_MOTORS_PER_MODULE));
  }

  private static RobotConfig createRobotConfig(
      double massKg,
      double moiKgMetersSquared,
      double wheelRadiusMeters,
      double wheelCof,
      double driveGearRatio,
      double driveCurrentLimitAmps,
      int driveMotorsPerModule) {
    ModuleConfig moduleConfig =
        new ModuleConfig(
            wheelRadiusMeters,
            DriveConstants.MAX_SPEED,
            wheelCof,
            DCMotor.getNEO(driveMotorsPerModule),
            driveGearRatio,
            driveCurrentLimitAmps,
            driveMotorsPerModule);
    return new RobotConfig(
        massKg, moiKgMetersSquared, moduleConfig, DriveConstants.MODULE_TRANSLATIONS);
  }
}
