package frc.robot;

import frc.robot.Commands.DriveCommand;
import frc.robot.Commands.DriveShooterAimCommand;
import frc.robot.Constants.SystemConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Controllers.DriverController;
import frc.robot.Controllers.GenericJoystickDriverController;
import frc.robot.Controllers.XboxDriverController;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.vision.Vision;
import frc.robot.Subsystems.vision.VisionIO;
import frc.robot.Subsystems.vision.VisionIOLimelight;
import frc.robot.Subsystems.vision.VisionIOSim;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.math.geometry.Pose2d;

/** Driver controls: field-relative driving on port 0. */
public final class RobotContainer {
  private final Drive drive = new Drive();
  private final Vision vision;
  private final DriverController controller;
  private final DriveCommand driveCommand;
  private final DriveShooterAimCommand shooterAimCommand;

  public RobotContainer() {
    controller =
        switch (SystemConstants.DRIVER_CONTROLLER_MODE) {
          case XBOX -> new XboxDriverController(SystemConstants.DRIVER_PORT);
          case GENERIC_JOYSTICK -> new GenericJoystickDriverController(SystemConstants.DRIVER_PORT);
        };
    driveCommand = new DriveCommand(drive, controller);
    shooterAimCommand = new DriveShooterAimCommand(drive, controller);
    drive.setDefaultCommand(driveCommand);
    Scheduler.getDefault().addPeriodic(drive::periodic);
    VisionIO visionIO =
        switch (SystemConstants.currentMode) {
          case REAL -> new VisionIOLimelight(VisionConstants.LIMELIGHT_HOSTNAME, drive::getHeading);
          case SIM -> new VisionIOSim();
          case REPLAY -> new VisionIO() {};
        };
    vision =
        new Vision(
            drive::addVisionMeasurement,
            drive::getAngularVelocityRadiansPerSec,
            SystemConstants.ODOMETRY_MODE,
            visionIO);
    Scheduler.getDefault().addPeriodic(vision::periodic);
    new Trigger(controller::aimToggle).toggleOnTrue(shooterAimCommand);
    new Trigger(controller::reset)
        .onTrue(
            Command.requiring(drive)
                .executing(coroutine -> drive.resetPose(new Pose2d()))
                .named("ResetDrivePose"));
  }

  DriveCommand defaultDriveCommand() {
    return driveCommand;
  }
}
