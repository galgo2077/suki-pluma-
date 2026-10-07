package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.Constants.DriveConstants;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

class SwerveMathTest {
  private static final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(DriveConstants.MODULE_TRANSLATIONS);

  @Test
  void fieldRelativeUsesHeading() {
    var zero = new ChassisVelocities(1, 0, 0).toRobotRelative(Rotation2d.ZERO);
    var ninety = new ChassisVelocities(1, 0, 0).toRobotRelative(Rotation2d.fromDegrees(90));
    assertEquals(1, zero.vx, 1e-9);
    assertEquals(0, zero.vy, 1e-9);
    assertEquals(0, ninety.vx, 1e-9);
    assertEquals(-1, ninety.vy, 1e-9);
  }

  @Test
  void optimizeReversesInsteadOfTurningAround() {
    var result =
        new SwerveModuleVelocity(1, Rotation2d.fromDegrees(190))
            .optimize(Rotation2d.fromDegrees(10));
    assertEquals(-1, result.velocity, 1e-9);
    assertEquals(10, result.angle.getDegrees(), 1e-9);
  }

  @Test
  void desaturationRespectsSimMaximum() {
    var raw =
        kinematics.toSwerveModuleVelocities(
            new ChassisVelocities(
                DriveConstants.MAX_SPEED, DriveConstants.MAX_SPEED, DriveConstants.MAX_OMEGA));
    var limited = SwerveDriveKinematics.desaturateWheelVelocities(raw, DriveConstants.MAX_SPEED);
    for (var state : limited)
      assertTrue(Math.abs(state.velocity) <= DriveConstants.MAX_SPEED + 1e-9);
  }

  @Test
  void zeroCommandIsStationary() {
    var states = kinematics.toSwerveModuleVelocities(new ChassisVelocities());
    assertEquals(4, states.length);
    for (var state : states) assertEquals(0, state.velocity, 1e-9);
  }

  @Test
  void resetKeepsPoseAndGyroCoherent() {
    var drive = new Drive();
    var reset = new Pose2d(1, 1, Rotation2d.ZERO);
    drive.resetPose(reset);
    assertEquals(reset, drive.pose());
    assertEquals(0, drive.heading().getRadians(), 1e-9);
  }
}
