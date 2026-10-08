package frc.robot.Subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.ShooterConstants;
import java.util.Optional;
import java.util.OptionalDouble;

/** Field-distance and future RPM calculations for the shooter. */
public final class ShooterCalculator {
  private static final double FIELD_LENGTH_METERS = Units.inchesToMeters(651.2);
  private static final double FIELD_WIDTH_METERS = Units.inchesToMeters(317.7);
  private static final double HUB_DISTANCE_FROM_ALLIANCE_WALL_METERS = Units.inchesToMeters(158.6);
  private static final Translation2d BLUE_HUB =
      new Translation2d(HUB_DISTANCE_FROM_ALLIANCE_WALL_METERS, FIELD_WIDTH_METERS / 2.0);
  private static final Translation2d RED_HUB =
      new Translation2d(
          FIELD_LENGTH_METERS - HUB_DISTANCE_FROM_ALLIANCE_WALL_METERS, FIELD_WIDTH_METERS / 2.0);
  private static final InterpolatingDoubleTreeMap DISTANCE_TO_RPM =
      new InterpolatingDoubleTreeMap();
  // TODO CALIBRATE: add Suki Pluma distance-to-RPM points and matching inclusive range limits.
  private static final double MIN_CALIBRATED_DISTANCE_METERS = Double.NaN;
  private static final double MAX_CALIBRATED_DISTANCE_METERS = Double.NaN;

  private ShooterCalculator() {}

  /** Returns no distance until both the robot pose and FMS alliance are known. */
  public static OptionalDouble distanceToHubMeters(Pose2d robotPose, Optional<Alliance> alliance) {
    return distanceToHubMeters(
        robotPose, alliance, ShooterConstants.SHOOTER_EXIT_FROM_ROBOT_CENTER_METERS);
  }

  /** Returns the horizontal shooter-exit-to-Hub distance for a known robot-relative exit offset. */
  static OptionalDouble distanceToHubMeters(
      Pose2d robotPose, Optional<Alliance> alliance, Translation2d shooterExitFromRobotCenter) {
    if (robotPose == null
        || alliance.isEmpty()
        || shooterExitFromRobotCenter == null
        || !Double.isFinite(robotPose.getX())
        || !Double.isFinite(robotPose.getY())
        || !Double.isFinite(shooterExitFromRobotCenter.getX())
        || !Double.isFinite(shooterExitFromRobotCenter.getY())) return OptionalDouble.empty();

    Translation2d hub = alliance.get() == Alliance.Blue ? BLUE_HUB : RED_HUB;
    Translation2d shooterExit =
        robotPose
            .getTranslation()
            .plus(shooterExitFromRobotCenter.rotateBy(robotPose.getRotation()));
    return OptionalDouble.of(shooterExit.getDistance(hub));
  }

  /** Calculates target flywheel RPM by linearly interpolating the calibrated Suki Pluma table. */
  public static double calculateRPM(double distanceMeters) {
    return calculateRPM(
        distanceMeters,
        MIN_CALIBRATED_DISTANCE_METERS,
        MAX_CALIBRATED_DISTANCE_METERS,
        DISTANCE_TO_RPM);
  }

  static double calculateRPM(
      double distanceMeters,
      double minCalibratedDistanceMeters,
      double maxCalibratedDistanceMeters,
      InterpolatingDoubleTreeMap distanceToRpm) {
    if (!Double.isFinite(distanceMeters)
        || !Double.isFinite(minCalibratedDistanceMeters)
        || !Double.isFinite(maxCalibratedDistanceMeters)
        || minCalibratedDistanceMeters > maxCalibratedDistanceMeters
        || distanceMeters < minCalibratedDistanceMeters
        || distanceMeters > maxCalibratedDistanceMeters) return Double.NaN;

    Double rpm = distanceToRpm.get(distanceMeters);
    return rpm != null && Double.isFinite(rpm) && rpm > 0.0 ? rpm : Double.NaN;
  }
}
