package frc.robot.Commands;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.DriveConstants;
import frc.robot.Controllers.DriverController;
import frc.robot.Subsystems.drive.Drive;
import org.junit.jupiter.api.Test;

class DriveCommandTest {
  @Test
  void stickCommandUsesNormalizedInputs() {
    var drive = new CapturingDrive();
    var command = new DriveCommand(drive, new Input(1, -1, 1));
    command.executeOnce();
    assertEquals(-DriveConstants.MAX_SPEED, drive.x, 1e-9);
    assertEquals(-DriveConstants.MAX_SPEED, drive.y, 1e-9);
    assertEquals(DriveConstants.MAX_OMEGA, drive.omega, 1e-9);

    new DriveCommand(drive, new Input(0, 0, 0)).executeOnce();
    assertEquals(0, drive.x, 1e-9);
    assertEquals(0, drive.y, 1e-9);
    assertEquals(0, drive.omega, 1e-9);
  }

  @Test
  void forwardIsInvertedOnlyForBlue() {
    assertEquals(-1, DriveCommand.forwardForAlliance(1, Alliance.Blue));
    assertEquals(1, DriveCommand.forwardForAlliance(1, Alliance.Red));
  }

  @Test
  void neutralStickDriftInsideDeadbandCommandsZero() {
    var drive = new CapturingDrive();
    new DriveCommand(drive, new Input(-0.139, 0.003, -0.082)).executeOnce();

    assertEquals(0, drive.x, 1e-9);
    assertEquals(0, drive.y, 1e-9);
    assertEquals(0, drive.omega, 1e-9);
  }

  private record Input(double forward, double strafe, double rotation) implements DriverController {
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
