package frc.robot.Constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Official 2026 REBUILT Hub target centers in the WPILib blue-origin field frame. */
public final class ShootingTargetConstants {
  private ShootingTargetConstants() {}

  private static final double HUB_CENTER_FROM_ALLIANCE_WALL = Units.inchesToMeters(158.6);
  private static final double FIELD_LENGTH = VisionConstants.APRILTAG_LAYOUT.getFieldLength();
  private static final double FIELD_WIDTH = VisionConstants.APRILTAG_LAYOUT.getFieldWidth();

  public static final Translation2d BLUE_HUB =
      new Translation2d(HUB_CENTER_FROM_ALLIANCE_WALL, FIELD_WIDTH / 2.0);
  public static final Translation2d RED_HUB =
      new Translation2d(FIELD_LENGTH - HUB_CENTER_FROM_ALLIANCE_WALL, FIELD_WIDTH / 2.0);

  public static Translation2d hubFor(Alliance alliance) {
    return alliance == Alliance.Red ? RED_HUB : BLUE_HUB;
  }
}
