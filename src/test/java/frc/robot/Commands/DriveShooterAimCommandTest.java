package frc.robot.Commands;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShootingTargetConstants;
import frc.robot.Controllers.DriverController;
import frc.robot.Subsystems.drive.Drive;
import org.junit.jupiter.api.Test;
import org.wpilib.driverstation.Alliance;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

class DriveShooterAimCommandTest {
  private static final Translation2d ORIGIN = Translation2d.ZERO;

  @Test
  void targetBearingsAndShooterOffsetUseFieldGeometry() {
    assertHeading(ORIGIN, new Translation2d(1, 0), Rotation2d.ZERO, 0);
    assertHeading(ORIGIN, new Translation2d(0, 1), Rotation2d.ZERO, 90);
    assertHeading(ORIGIN, new Translation2d(-1, 0), Rotation2d.ZERO, 180);
    assertHeading(ORIGIN, new Translation2d(0, -1), Rotation2d.ZERO, -90);
    assertHeading(ORIGIN, new Translation2d(0, 1), Rotation2d.fromDegrees(180), -90);
  }

  @Test
  void headingUpdatesAsRobotMovesAroundTarget() {
    Translation2d target = Translation2d.ZERO;
    assertHeading(new Translation2d(0, -1), target, 90);
    assertHeading(new Translation2d(-1, 0), target, 0);
    assertHeading(new Translation2d(0, 1), target, -90);
    assertHeading(new Translation2d(1, 0), target, 180);
  }

  @Test
  void zeroDistanceHasNoDesiredHeading() {
    assertTrue(
        DriveShooterAimCommand.desiredHeading(
                new Pose2d(ORIGIN, Rotation2d.ZERO), ORIGIN, Rotation2d.ZERO)
            .isEmpty());
  }

  @Test
  void retainsFieldRelativeTranslationAndIgnoresRotationStick() {
    var drive = new CapturingDrive(new Pose2d(0, 0, Rotation2d.fromDegrees(90)));
    new DriveShooterAimCommand(drive, new Input(1, 0, 0)).executeOnce();
    double omegaWithoutStick = drive.omega;
    assertEquals(DriveConstants.MAX_SPEED, drive.x, 1e-9);
    assertEquals(0, drive.y, 1e-9);
    assertNotEquals(0, omegaWithoutStick);

    new DriveShooterAimCommand(drive, new Input(1, 0, 1)).executeOnce();
    assertEquals(DriveConstants.MAX_SPEED, drive.x, 1e-9);
    assertEquals(0, drive.y, 1e-9);
    assertEquals(omegaWithoutStick, drive.omega, 1e-9);
  }

  @Test
  void continuousControllerUsesShortestPathAndCommandsAreMutuallyExclusive() {
    Translation2d target = DriveShooterAimCommand.shootingTarget();
    var drive =
        new CapturingDrive(
            new Pose2d(target.getX() - 1, target.getY(), Rotation2d.fromDegrees(359)));
    new DriveShooterAimCommand(drive, new Input(0, 0, 0)).executeOnce();
    assertTrue(drive.omega > 0);
    assertTrue(drive.omega < Math.toRadians(10));
    var stick = new DriveCommand(drive, new Input(0, 0, 0));
    var aim = new DriveShooterAimCommand(drive, new Input(0, 0, 0));
    assertTrue(stick.conflictsWith(aim));
  }

  @Test
  void allianceTargetsAreOfficialHubMirrors() {
    assertEquals(ShootingTargetConstants.BLUE_HUB, ShootingTargetConstants.hubFor(Alliance.BLUE));
    assertEquals(ShootingTargetConstants.RED_HUB, ShootingTargetConstants.hubFor(Alliance.RED));
    assertEquals(
        ShootingTargetConstants.BLUE_HUB.getX() + ShootingTargetConstants.RED_HUB.getX(),
        org.wpilib.fields.Fields.FRC_2026_REBUILT_WELDED.length,
        1e-9);
  }

  private static void assertHeading(
      Translation2d robot, Translation2d target, double expectedDegrees) {
    assertHeading(robot, target, Rotation2d.ZERO, expectedDegrees);
  }

  private static void assertHeading(
      Translation2d robot, Translation2d target, Rotation2d offset, double expectedDegrees) {
    Rotation2d heading =
        DriveShooterAimCommand.desiredHeading(new Pose2d(robot, Rotation2d.ZERO), target, offset)
            .orElseThrow();
    assertEquals(Rotation2d.fromDegrees(expectedDegrees), heading);
  }

  private record Input(double forward, double strafe, double rotation) implements DriverController {
    @Override
    public boolean reset() {
      return false;
    }
  }

  private static final class CapturingDrive extends Drive {
    private Pose2d pose;
    double x;
    double y;
    double omega;

    CapturingDrive(Pose2d pose) {
      this.pose = pose;
    }

    @Override
    public Pose2d getPose() {
      return pose;
    }

    @Override
    public void driveFieldRelative(double x, double y, double omega) {
      this.x = x;
      this.y = y;
      this.omega = omega;
    }
  }
}
