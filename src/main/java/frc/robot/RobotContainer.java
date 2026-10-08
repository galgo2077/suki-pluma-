package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.DriveCommand;
import frc.robot.Commands.DriveShooterAimCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SystemConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Controllers.DriverController;
import frc.robot.Controllers.GenericJoystickDriverController;
import frc.robot.Controllers.XboxDriverController;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.drive.DriveAutonomous;
import frc.robot.Subsystems.shooter.Shooter;
import frc.robot.Subsystems.shooter.ShooterCalculator;
import frc.robot.Subsystems.shooter.ShooterIO;
import frc.robot.Subsystems.shooter.ShooterIOReal;
import frc.robot.Subsystems.shooter.ShooterIOSim;
import frc.robot.Subsystems.vision.LimelightCamera;
import frc.robot.Subsystems.vision.Vision;
import frc.robot.Subsystems.vision.VisionIO;
import frc.robot.Subsystems.vision.VisionIOLimelight;
import frc.robot.Subsystems.vision.VisionIOSim;
import java.util.OptionalDouble;
import java.util.function.BooleanSupplier;

/** Owns the wheel/gyro pose estimator and its AprilTag correction source. */
public final class RobotContainer {
  private final Drive drive = new Drive();

  @SuppressWarnings("unused")
  private final Vision vision;

  @SuppressWarnings("unused")
  private final Shooter shooter;

  private final DriverController controller;
  private final DriveCommand driveCommand;
  private final DriveShooterAimCommand shooterAimCommand;
  private SendableChooser<Command> autoChooser;

  private final Joystick shooterJoystick = new Joystick(SystemConstants.JOYSTICK_PORT);
  private final CommandXboxController shooterXbox =
      new CommandXboxController(SystemConstants.XBOX_PORT);

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
    ShooterIO shooterIO =
        switch (SystemConstants.currentMode) {
          case REAL -> realShooterIO();
          case SIM -> new ShooterIOSim();
          case REPLAY -> new ShooterIO() {};
        };
    shooter =
        new Shooter(
            shooterIO,
            SystemConstants.currentMode == SystemConstants.Mode.SIM
                ? ShooterConstants.SIM_RPM_TOLERANCE
                : ShooterConstants.RPM_TOLERANCE,
            SystemConstants.currentMode == SystemConstants.Mode.SIM
                ? ShooterConstants.SIM_FEEDER_OUTPUT
                : ShooterConstants.REAL_FEEDER_OUTPUT);
    boolean realConfigurationValid = ShooterConstants.realHardwareConfigurationValid();
    boolean shootingPermitted =
        shootingPermitted(SystemConstants.currentMode, realConfigurationValid);
    shooter.setConfigurationStatus(
        SystemConstants.currentMode == SystemConstants.Mode.REAL, realConfigurationValid);
    if (shootingPermitted)
      combinedShootTrigger(
              () -> shooterJoystick.getRawButton(1),
              shooterXbox.rightTrigger(SystemConstants.XBOX_TRIGGER_THRESHOLD))
          .whileTrue(shooter.shootCommand(this::shooterTargetRpm));
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

  static Trigger combinedShootTrigger(BooleanSupplier joystickTrigger, Trigger xboxRightTrigger) {
    return new Trigger(joystickTrigger).or(xboxRightTrigger);
  }

  static boolean shootingPermitted(SystemConstants.Mode mode, boolean realConfigurationValid) {
    return mode == SystemConstants.Mode.SIM
        || (mode == SystemConstants.Mode.REAL && realConfigurationValid);
  }

  private double shooterTargetRpm() {
    OptionalDouble distance =
        ShooterCalculator.distanceToHubMeters(drive.getPose(), DriverStation.getAlliance());
    shooter.setDistanceToHubMeters(distance.orElse(Double.NaN));
    double calculatedRpm =
        distance.isPresent() ? ShooterCalculator.calculateRPM(distance.getAsDouble()) : Double.NaN;
    shooter.setCalculationValid(Double.isFinite(calculatedRpm) && calculatedRpm > 0.0);

    // The explicit SIM RPM mode remains available while production calibration is incomplete.
    return SystemConstants.currentMode == SystemConstants.Mode.SIM
        ? ShooterConstants.SIM_TEST_TARGET_RPM
        : calculatedRpm;
  }

  private static ShooterIO realShooterIO() {
    if (ShooterConstants.realHardwareConfigurationValid()) return new ShooterIOReal();
    DriverStation.reportError(
        "Shooter disabled: required REAL hardware configuration is undefined", false);
    return new ShooterIO() {};
  }
}
