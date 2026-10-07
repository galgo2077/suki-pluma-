package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.Constants.DriveConstants;
import frc.robot.controllers.DriverController;
import frc.robot.subsystems.drive.Drive;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

class DriveCommandTest {
  @Test
  void stickCommandUsesNormalizedInputs() {
    var drive = new CapturingDrive();
    var command = new DriveCommand(drive, new Input(1, -1, 1, -1));
    command.executeOnce();
    assertEquals(DriveConstants.MAX_SPEED, drive.x, 1e-9);
    assertEquals(-DriveConstants.MAX_SPEED, drive.y, 1e-9);
    assertEquals(DriveConstants.MAX_OMEGA, drive.omega, 1e-9);

    new DriveCommand(drive, new Input(0, 0, 0, -1)).executeOnce();
    assertEquals(0, drive.x, 1e-9);
    assertEquals(0, drive.y, 1e-9);
    assertEquals(0, drive.omega, 1e-9);
  }

  @Test
  void headingMappingUsesOnlyCardinals() {
    assertEquals(0, DriveHeadingCommand.headingForPov(0).orElseThrow().getDegrees(), 1e-9);
    assertEquals(90, DriveHeadingCommand.headingForPov(90).orElseThrow().getDegrees(), 1e-9);
    assertEquals(180, DriveHeadingCommand.headingForPov(180).orElseThrow().getDegrees(), 1e-9);
    assertEquals(Rotation2d.fromDegrees(270), DriveHeadingCommand.headingForPov(270).orElseThrow());
    assertTrue(DriveHeadingCommand.headingForPov(45).isEmpty());
  }

  @Test
  void headingUsesShortestContinuousPath() {
    assertHeadingDirection(359, 0, 1);
    assertHeadingDirection(1, 0, -1);
    assertHeadingDirection(179, 180, 1);
  }

  @Test
  void headingCommandRetainsFieldTranslation() {
    var drive = new CapturingDrive();
    new DriveHeadingCommand(drive, new Input(1, 0, 0, 90)).executeOnce();
    assertEquals(DriveConstants.MAX_SPEED, drive.x, 1e-9);
    assertEquals(0, drive.y, 1e-9);
    assertTrue(drive.omega > 0);
  }

  @Test
  void commandsConflictAndStickIsDefault() {
    var drive = new CapturingDrive();
    var stick = new DriveCommand(drive, new Input(0, 0, 0, -1));
    var heading = new DriveHeadingCommand(drive, new Input(0, 0, 0, -1));
    assertEquals(stick.requirements(), heading.requirements());
    assertTrue(stick.conflictsWith(heading));
  }

  private static void assertHeadingDirection(double currentDegrees, int pov, int expectedSign) {
    var drive = new CapturingDrive();
    drive.resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(currentDegrees)));
    new DriveHeadingCommand(drive, new Input(0, 0, 0, pov)).executeOnce();
    assertEquals(expectedSign, Math.signum(drive.omega));
  }

  private record Input(double forward, double strafe, double rotation, int pov)
      implements DriverController {
    @Override
    public boolean reset() {
      return false;
    }
  }

  private static final class CapturingDrive extends Drive {
    double x;
    double y;
    double omega;

    @Override
    public void driveFieldRelative(double x, double y, double omega) {
      this.x = x;
      this.y = y;
      this.omega = omega;
    }
  }
}
