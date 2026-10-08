package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SystemConstants;
import frc.robot.Controllers.DriverController;
import frc.robot.Subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

/** Default field-oriented stick driving. */
public final class DriveCommand extends Command {
  private final Drive drive;
  private final DriverController controller;

  public DriveCommand(Drive drive, DriverController controller) {
    this.drive = drive;
    this.controller = controller;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    executeOnce();
  }

  void executeOnce() {
    double forward =
        forwardForAlliance(controller.forward(), DriverStation.getAlliance().orElse(Alliance.Blue));
    double strafe = controller.strafe();
    double rotation = controller.rotation();
    Logger.recordOutput("Drive/ControlMode", "STICK");
    Logger.recordOutput("Driver/Forward", forward);
    Logger.recordOutput("Driver/Strafe", strafe);
    Logger.recordOutput("Driver/Rotation", rotation);
    drive.driveFieldRelative(
        MathUtil.applyDeadband(forward, SystemConstants.DRIVER_DEADBAND) * DriveConstants.MAX_SPEED,
        MathUtil.applyDeadband(strafe, SystemConstants.DRIVER_DEADBAND) * DriveConstants.MAX_SPEED,
        MathUtil.applyDeadband(rotation, SystemConstants.DRIVER_DEADBAND)
            * DriveConstants.MAX_OMEGA);
  }

  static double forwardForAlliance(double forward, Alliance alliance) {
    return alliance == Alliance.Blue ? -forward : forward;
  }
}
