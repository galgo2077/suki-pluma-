package frc.robot.Subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PathplannerConstants;
import frc.robot.Constants.SystemConstants;
import org.littletonrobotics.junction.Logger;

/** The single integration point between PathPlanner and the existing Drive subsystem. */
public final class DriveAutonomous {
  private static final String MISSING_MEASUREMENTS = "Missing DriveConstants measurements";

  private DriveAutonomous() {}

  /**
   * Configures AutoBuilder once when the drivetrain's verified physical configuration is complete.
   */
  public static synchronized void configure(Drive drive) {
    if (AutoBuilder.isConfigured()) {
      Logger.recordOutput("Drive/AutoBuilder/Ready", true);
      publishStatus(true, "Ready");
      return;
    }

    (SystemConstants.currentMode == SystemConstants.Mode.SIM
            ? java.util.Optional.of(PathplannerConstants.simulationRobotConfig())
            : PathplannerConstants.robotConfig())
        .ifPresentOrElse(
            config -> configure(drive, config),
            () -> {
              Logger.recordOutput("Drive/AutoBuilder/Ready", false);
              publishStatus(false, MISSING_MEASUREMENTS);
              DriverStation.reportWarning(
                  "PathPlanner AutoBuilder unavailable: " + MISSING_MEASUREMENTS, false);
            });
  }

  static synchronized void configure(Drive drive, RobotConfig config) {
    if (AutoBuilder.isConfigured()) {
      publishStatus(true, "Ready");
      return;
    }

    AutoBuilder.configure(
        drive::getPose,
        drive::resetPose,
        drive::getRobotRelativeSpeeds,
        drive::driveRobotRelative,
        new PPHolonomicDriveController(
            PathplannerConstants.TRANSLATION_PID, PathplannerConstants.ROTATION_PID),
        config,
        DriveAutonomous::shouldFlipPath,
        drive);
    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> Logger.recordOutput("Drive/PathPlanner/TargetPose", pose));
    PathPlannerLogging.setLogActivePathCallback(
        poses -> {
          drive.setPathPlannerTrajectory(poses);
          Logger.recordOutput(
              "Drive/PathPlanner/ActiveTrajectory",
              poses.toArray(new edu.wpi.first.math.geometry.Pose2d[0]));
        });
    Logger.recordOutput("Drive/AutoBuilder/Ready", true);
    publishStatus(true, "Ready");
  }

  private static void publishStatus(boolean ready, String status) {
    SmartDashboard.putBoolean("AutoBuilder Ready", ready);
    SmartDashboard.putString("AutoBuilder Status", status);
  }

  public static boolean isConfigured() {
    return AutoBuilder.isConfigured();
  }

  static boolean shouldFlipPath() {
    return DriverStation.getAlliance()
        .map(alliance -> alliance == DriverStation.Alliance.Red)
        .orElse(false);
  }
}
