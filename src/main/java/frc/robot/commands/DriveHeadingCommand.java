package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.controllers.DriverController;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.MathUtil;

/** Field-oriented translation while holding an absolute field heading selected by the POV. */
public final class DriveHeadingCommand implements Command {
  private final Drive drive;
  private final DriverController controller;
  private final PIDController headingController =
      new PIDController(
          DriveConstants.HEADING_KP, DriveConstants.HEADING_KI, DriveConstants.HEADING_KD);
  private Rotation2d target;

  public DriveHeadingCommand(Drive drive, DriverController controller) {
    this.drive = drive;
    this.controller = controller;
    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void run(Coroutine coroutine) {
    target = drive.pose().getRotation();
    while (true) {
      executeOnce();
      coroutine.yield();
    }
  }

  void executeOnce() {
    if (target == null) target = drive.pose().getRotation();
    headingForPov(controller.pov()).ifPresent(value -> target = value);
    double current = drive.pose().getRotation().getRadians();
    double omega =
        Math.clamp(
            headingController.calculate(current, target.getRadians()),
            -DriveConstants.MAX_OMEGA,
            DriveConstants.MAX_OMEGA);
    double forward = controller.forward();
    double strafe = controller.strafe();
    Logger.recordOutput("Drive/ControlMode", "HEADING");
    Logger.recordOutput("Driver/Forward", forward);
    Logger.recordOutput("Driver/Strafe", strafe);
    Logger.recordOutput("Driver/Rotation", controller.rotation());
    Logger.recordOutput("Driver/POV", controller.pov());
    Logger.recordOutput("Drive/Heading/Current", current);
    Logger.recordOutput("Drive/Heading/Target", target.getRadians());
    Logger.recordOutput("Drive/Heading/Error", headingController.getError());
    Logger.recordOutput("Drive/Heading/OmegaCommand", omega);
    drive.driveFieldRelative(
        MathUtil.applyDeadband(forward, 0.10) * DriveConstants.MAX_SPEED,
        MathUtil.applyDeadband(strafe, 0.10) * DriveConstants.MAX_SPEED,
        omega);
  }

  public static Optional<Rotation2d> headingForPov(int pov) {
    return switch (pov) {
      case 0 -> Optional.of(Rotation2d.ZERO);
      case 90 -> Optional.of(Rotation2d.fromDegrees(90));
      case 180 -> Optional.of(Rotation2d.fromDegrees(180));
      case 270 -> Optional.of(Rotation2d.fromDegrees(270));
      default -> Optional.empty();
    };
  }

  @Override
  public String name() {
    return "DriveHeadingCommand";
  }

  @Override
  public Set<Mechanism> requirements() {
    return Set.of(drive);
  }
}
