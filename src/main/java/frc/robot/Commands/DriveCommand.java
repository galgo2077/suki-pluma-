package frc.robot.Commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SystemConstants;
import frc.robot.Controllers.DriverController;
import frc.robot.Subsystems.drive.Drive;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.util.MathUtil;

/** Default field-oriented stick driving. */
public final class DriveCommand implements Command {
  private final Drive drive;
  private final DriverController controller;

  public DriveCommand(Drive drive, DriverController controller) {
    this.drive = drive;
    this.controller = controller;
  }

  @Override
  public void run(Coroutine coroutine) {
    while (true) {
      executeOnce();
      coroutine.yield();
    }
  }

  void executeOnce() {
    double forward = controller.forward();
    double strafe = controller.strafe();
    double rotation = controller.rotation();
    Logger.recordOutput("Drive/ControlMode", "STICK");
    Logger.recordOutput("Driver/Forward", forward);
    Logger.recordOutput("Driver/Strafe", strafe);
    Logger.recordOutput("Driver/Rotation", rotation);
    Logger.recordOutput("Driver/POV", controller.pov());
    drive.driveFieldRelative(
        MathUtil.applyDeadband(forward, SystemConstants.DRIVER_DEADBAND) * DriveConstants.MAX_SPEED,
        MathUtil.applyDeadband(strafe, SystemConstants.DRIVER_DEADBAND) * DriveConstants.MAX_SPEED,
        MathUtil.applyDeadband(rotation, SystemConstants.DRIVER_DEADBAND)
            * DriveConstants.MAX_OMEGA);
  }

  @Override
  public String name() {
    return "DriveCommand";
  }

  @Override
  public Set<Mechanism> requirements() {
    return Set.of(drive);
  }
}
