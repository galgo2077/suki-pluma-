package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShootingTargetConstants;
import frc.robot.Constants.SystemConstants;
import frc.robot.Controllers.DriverController;
import frc.robot.Subsystems.drive.Drive;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/** Field-relative driver translation while the fixed shooter tracks the alliance Hub. */
public final class DriveShooterAimCommand extends Command {
  // TODO CALIBRATE: fixed shooter forward offset from robot +X, rad.
  public static final Rotation2d SHOOTER_FORWARD_OFFSET = new Rotation2d();
  private static final double MIN_TARGET_DISTANCE_METERS = 1e-6;

  private final Drive drive;
  private final DriverController controller;
  private final PIDController headingController =
      new PIDController(
          DriveConstants.HEADING_KP, DriveConstants.HEADING_KI, DriveConstants.HEADING_KD);

  public DriveShooterAimCommand(Drive drive, DriverController controller) {
    this.drive = drive;
    this.controller = controller;
    addRequirements(drive);
    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void execute() {
    executeOnce();
  }

  void executeOnce() {
    Pose2d robotPose = drive.getPose();
    Translation2d target = shootingTarget();
    Rotation2d current = robotPose.getRotation();
    Optional<Rotation2d> desired = desiredHeading(robotPose, target, SHOOTER_FORWARD_OFFSET);
    Rotation2d desiredHeading = desired.orElse(current);
    double omega =
        desired
            .map(
                heading ->
                    MathUtil.clamp(
                        headingController.calculate(current.getRadians(), heading.getRadians()),
                        -DriveConstants.MAX_OMEGA,
                        DriveConstants.MAX_OMEGA))
            .orElse(0.0);
    double forward = controller.forward();
    double strafe = controller.strafe();

    Logger.recordOutput("Drive/ControlMode", "SHOOTER_AIM");
    Logger.recordOutput("Drive/Aim/Enabled", true);
    Logger.recordOutput("Drive/Aim/TargetPose", new Pose2d(target, new Rotation2d()));
    Logger.recordOutput(
        "Drive/Aim/TargetBearing", desiredHeading.plus(SHOOTER_FORWARD_OFFSET).getRadians());
    Logger.recordOutput("Drive/Aim/DesiredRobotHeading", desiredHeading.getRadians());
    Logger.recordOutput("Drive/Aim/CurrentRobotHeading", current.getRadians());
    Logger.recordOutput("Drive/Aim/HeadingError", headingController.getError());
    Logger.recordOutput("Drive/Aim/OmegaCommand", omega);
    Logger.recordOutput(
        "Drive/Aim/AtTarget", desired.isPresent() && headingController.atSetpoint());
    Logger.recordOutput(
        "Drive/Aim/ShooterFacingPose",
        new Pose2d(robotPose.getTranslation(), desiredHeading.plus(SHOOTER_FORWARD_OFFSET)));
    drive.driveFieldRelative(
        MathUtil.applyDeadband(forward, SystemConstants.DRIVER_DEADBAND) * DriveConstants.MAX_SPEED,
        MathUtil.applyDeadband(strafe, SystemConstants.DRIVER_DEADBAND) * DriveConstants.MAX_SPEED,
        omega);
  }

  static Optional<Rotation2d> desiredHeading(
      Pose2d robotPose, Translation2d target, Rotation2d shooterForwardOffset) {
    Translation2d displacement = target.minus(robotPose.getTranslation());
    if (displacement.getNorm() < MIN_TARGET_DISTANCE_METERS) return Optional.empty();
    return Optional.of(displacement.getAngle().minus(shooterForwardOffset));
  }

  static Translation2d shootingTarget() {
    return ShootingTargetConstants.hubFor(DriverStation.getAlliance().orElse(Alliance.Blue));
  }

  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("Drive/Aim/Enabled", false);
  }
}
