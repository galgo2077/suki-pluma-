package frc.robot;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveHeadingCommand;
import frc.robot.controllers.DriverController;
import frc.robot.controllers.GenericJoystickDriverController;
import frc.robot.controllers.XboxDriverController;
import frc.robot.subsystems.drive.Drive;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.math.geometry.Pose2d;

/** Driver controls: field-relative driving on port 0. */
public final class RobotContainer {
  private enum ControllerMode {
    XBOX,
    GENERIC_JOYSTICK
  }

  private static final ControllerMode DRIVER_MODE = ControllerMode.XBOX;
  private static final int DRIVER_PORT = 0;

  private final Drive drive = new Drive();
  private final DriverController controller;
  private final DriveCommand driveCommand;

  public RobotContainer() {
    controller =
        switch (DRIVER_MODE) {
          case XBOX -> new XboxDriverController(DRIVER_PORT);
          case GENERIC_JOYSTICK -> new GenericJoystickDriverController(DRIVER_PORT);
        };
    driveCommand = new DriveCommand(drive, controller);
    drive.setDefaultCommand(driveCommand);
    Scheduler.getDefault().addPeriodic(drive::periodic);
    new Trigger(() -> controller.pov() != DriverController.POV_CENTER)
        .whileTrue(new DriveHeadingCommand(drive, controller));
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
