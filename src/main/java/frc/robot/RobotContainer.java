package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.DriveCommand;
import frc.robot.Commands.DriveShooterAimCommand;
import frc.robot.Constants.SystemConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Controllers.DriverController;
import frc.robot.Controllers.GenericJoystickDriverController;
import frc.robot.Controllers.XboxDriverController;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.drive.DriveAutonomous;
import frc.robot.Subsystems.vision.LimelightCamera;
import frc.robot.Subsystems.vision.Vision;
import frc.robot.Subsystems.vision.VisionIO;
import frc.robot.Subsystems.vision.VisionIOLimelight;
import frc.robot.Subsystems.vision.VisionIOSim;

/** Driver controls: field-relative driving on port 0. */
public final class RobotContainer {
  private final Drive drive = new Drive();

  @SuppressWarnings("unused")
  private final Vision vision;

  private final DriverController controller;
  private final DriveCommand driveCommand;
  private final DriveShooterAimCommand shooterAimCommand;
  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    controller =
        switch (SystemConstants.DRIVER_CONTROLLER_MODE) {
          case XBOX -> new XboxDriverController(SystemConstants.DRIVER_PORT);
          case GENERIC_JOYSTICK -> new GenericJoystickDriverController(SystemConstants.DRIVER_PORT);
        };
    driveCommand = new DriveCommand(drive, controller);
    shooterAimCommand = new DriveShooterAimCommand(drive, controller);
    drive.setDefaultCommand(driveCommand);
    VisionIO visionIO =
        switch (SystemConstants.currentMode) {
          case REAL -> new VisionIOLimelight(VisionConstants.LIMELIGHT_HOSTNAME);
          case SIM -> new VisionIOSim();
          case REPLAY -> new VisionIO() {};
        };
    if (SystemConstants.currentMode != SystemConstants.Mode.REPLAY)
      LimelightCamera.register("Limelight", VisionConstants.LIMELIGHT_STREAM_URL);
    vision = new Vision(drive::addVisionMeasurement, SystemConstants.ODOMETRY_MODE, visionIO);
    new Trigger(controller::aimToggle).toggleOnTrue(shooterAimCommand);
    new Trigger(controller::reset)
        .onTrue(
            new InstantCommand(() -> drive.resetPose(new Pose2d()), drive)
                .withName("ResetDrivePose"));
    configureAutoChooser();
  }

  private void configureAutoChooser() {
    if (autoChooser != null) return;
    if (!DriveAutonomous.isConfigured()) {
      DriverStation.reportWarning(
          "PathPlanner auto chooser unavailable: AutoBuilder is not configured", false);
      return;
    }
    try {
      autoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", autoChooser);
    } catch (RuntimeException e) {
      DriverStation.reportError(
          "PathPlanner auto chooser failed: " + e.getMessage(), e.getStackTrace());
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser == null ? null : autoChooser.getSelected();
  }

  DriveCommand defaultDriveCommand() {
    return driveCommand;
  }
}
