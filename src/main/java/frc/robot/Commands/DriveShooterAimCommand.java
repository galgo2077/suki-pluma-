package frc.robot.Commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShootingTargetConstants;
import frc.robot.Constants.SystemConstants;
import frc.robot.Controllers.DriverController;
import frc.robot.Subsystems.drive.Drive;
import java.util.Optional;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.MathUtil;

/** Field-relative driver translation while the fixed shooter tracks the alliance Hub. */
public final class DriveShooterAimCommand implements Command {
  // TODO CALIBRATE: verify fixed shooter forward offset relative to robot +X
  public static final Rotation2d SHOOTER_FORWARD_OFFSET = Rotation2d.ZERO;
  private static final double MIN_TARGET_DISTANCE_METERS = 1e-6;

  private final Drive drive;
  private final DriverController controller;
  private final PIDController headingController =
      new PIDController(
          DriveConstants.HEADING_KP, DriveConstants.HEADING_KI, DriveConstants.HEADING_KD);

  public DriveShooterAimCommand(Drive drive, DriverController controller) {
    this.drive = drive;
    this.controller = controller;
    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void run(Coroutine coroutine) {
    while (true) {
      executeOnce();
      coroutine.yield();
    }
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
                    Math.clamp(
                        headingController.calculate(current.getRadians(), heading.getRadians()),
                        -DriveConstants.MAX_OMEGA,
                        DriveConstants.MAX_OMEGA))
            .orElse(0.0);
    double forward = controller.forward();
    double strafe = controller.strafe();

    Logger.recordOutput("Drive/ControlMode", "SHOOTER_AIM");
    Logger.recordOutput("Drive/Aim/Enabled", true);
    Logger.recordOutput("Drive/Aim/TargetPose", new Pose2d(target, Rotation2d.ZERO));
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
    return displacement.getAngle().map(angle -> angle.minus(shooterForwardOffset));
  }

  static Translation2d shootingTarget() {
    return ShootingTargetConstants.hubFor(DriverStationBackend.getAlliance().orElse(Alliance.BLUE));
  }

  @Override
  public void onCancel() {
    Logger.recordOutput("Drive/Aim/Enabled", false);
  }

  @Override
  public String name() {
    return "DriveShooterAimCommand";
  }

  @Override
  public Set<Mechanism> requirements() {
    return Set.of(drive);
  }
}
