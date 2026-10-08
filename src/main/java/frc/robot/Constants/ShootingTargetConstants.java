package frc.robot.Constants;

import org.wpilib.driverstation.Alliance;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.Units;

/** Official 2026 REBUILT Hub target centers in the WPILib blue-origin field frame. */
public final class ShootingTargetConstants {
  private ShootingTargetConstants() {}

  private static final double HUB_CENTER_FROM_ALLIANCE_WALL = Units.inchesToMeters(158.6);
  private static final double FIELD_LENGTH = Fields.FRC_2026_REBUILT_WELDED.length;
  private static final double FIELD_WIDTH = Fields.FRC_2026_REBUILT_WELDED.width;

  public static final Translation2d BLUE_HUB =
      new Translation2d(HUB_CENTER_FROM_ALLIANCE_WALL, FIELD_WIDTH / 2.0);
  public static final Translation2d RED_HUB =
      new Translation2d(FIELD_LENGTH - HUB_CENTER_FROM_ALLIANCE_WALL, FIELD_WIDTH / 2.0);

  public static Translation2d hubFor(Alliance alliance) {
    return alliance == Alliance.RED ? RED_HUB : BLUE_HUB;
  }
}
