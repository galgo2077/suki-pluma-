package frc.robot.Subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import org.junit.jupiter.api.Test;

class SwerveMathTest {
  private static final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(DriveConstants.MODULE_TRANSLATIONS);

  @Test
  void fieldRelativeUsesHeading() {
    var zero = ChassisSpeeds.fromFieldRelativeSpeeds(1, 0, 0, new Rotation2d());
    var ninety = ChassisSpeeds.fromFieldRelativeSpeeds(1, 0, 0, Rotation2d.fromDegrees(90));
    assertEquals(1, zero.vxMetersPerSecond, 1e-9);
    assertEquals(0, zero.vyMetersPerSecond, 1e-9);
    assertEquals(0, ninety.vxMetersPerSecond, 1e-9);
    assertEquals(-1, ninety.vyMetersPerSecond, 1e-9);
  }

  @Test
  void optimizeReversesInsteadOfTurningAround() {
    var result = new SwerveModuleState(1, Rotation2d.fromDegrees(190));
    result.optimize(Rotation2d.fromDegrees(10));
    assertEquals(-1, result.speedMetersPerSecond, 1e-9);
    assertEquals(10, result.angle.getDegrees(), 1e-9);
  }

  @Test
  void desaturationRespectsSimMaximum() {
    var raw =
        kinematics.toSwerveModuleStates(
            new ChassisSpeeds(
                DriveConstants.MAX_SPEED, DriveConstants.MAX_SPEED, DriveConstants.MAX_OMEGA));
    SwerveDriveKinematics.desaturateWheelSpeeds(raw, DriveConstants.MAX_SPEED);
    for (var state : raw)
      assertTrue(Math.abs(state.speedMetersPerSecond) <= DriveConstants.MAX_SPEED + 1e-9);
  }

  @Test
  void zeroCommandIsStationary() {
    var states = kinematics.toSwerveModuleStates(new ChassisSpeeds());
    assertEquals(4, states.length);
    for (var state : states) assertEquals(0, state.speedMetersPerSecond, 1e-9);
  }

  @Test
  void resetKeepsPoseAndGyroCoherent() {
    var drive = new Drive();
    var reset = new Pose2d(1, 1, new Rotation2d());
    drive.resetPose(reset);
    assertEquals(reset, drive.getPose());
    assertEquals(0, drive.getHeading().getRadians(), 1e-9);
  }

  @Test
  void publishesEstimatedPoseToTheSingleSwerveField() {
    var drive = new Drive();
    var pose = new Pose2d(1, 2, Rotation2d.fromDegrees(90));
    drive.resetPose(pose);
    drive.periodic();
    SmartDashboard.updateValues();

    var field = NetworkTableInstance.getDefault().getTable("SmartDashboard/Swerve Field");
    assertTrue(field.getTopic(".type").exists());
    double[] robotPose = field.getDoubleArrayTopic("Robot").subscribe(new double[0]).get();
    assertEquals(1, robotPose[0], 1e-9);
    assertEquals(2, robotPose[1], 1e-9);
    assertEquals(90, robotPose[2], 1e-9);
  }
}
