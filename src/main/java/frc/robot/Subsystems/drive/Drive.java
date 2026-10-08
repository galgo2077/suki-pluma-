package frc.robot.Subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.Odometry.Gyro.GyroIOInputsAutoLogged;
import frc.robot.Subsystems.Odometry.Gyro.GyroIOSim;
import frc.robot.Subsystems.drive.Motor.module.Module;
import frc.robot.Subsystems.drive.Motor.module.ModuleIOSim;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private final Module[] modules = {
    // create sim modules
    new Module("FL", new ModuleIOSim()),
    new Module("FR", new ModuleIOSim()),
    new Module("BL", new ModuleIOSim()),
    new Module("BR", new ModuleIOSim())
  };
  // create gyro
  private final GyroIOSim gyro = new GyroIOSim();
  private final GyroIOInputsAutoLogged gyroIn = new GyroIOInputsAutoLogged();

  // swerev odometrics pose
  private final SwerveDriveKinematics kin =
      new SwerveDriveKinematics(DriveConstants.MODULE_TRANSLATIONS);
  private final SwerveDrivePoseEstimator estimator =
      new SwerveDrivePoseEstimator(kin, new Rotation2d(), positions(), new Pose2d());

  private ChassisSpeeds command = new ChassisSpeeds();
  private SwerveModuleState[] desired = new SwerveModuleState[4];
  private final Field2d field = new Field2d();

  public Drive() {
    for (int i = 0; i < 4; i++) desired[i] = new SwerveModuleState();
    SmartDashboard.putData("Swerve Field", field);
    DriveAutonomous.configure(this);
  }

  private SwerveModulePosition[] positions() {
    SwerveModulePosition[] a = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) a[i] = modules[i].position();
    return a;
  }

  public void driveFieldRelative(double x, double y, double omega) {
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, omega, gyroIn.yaw));
    Logger.recordOutput("Drive/CommandFieldRelative", new ChassisSpeeds(x, y, omega));
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    command = speeds;
    desired = kin.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(desired, DriveConstants.MAX_SPEED);
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(desired[i]);
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < 4; i++) {
      modules[i].periodic();
    }
    gyro.setYaw(
        gyroIn.yaw.plus(
            new Rotation2d(
                command.omegaRadiansPerSecond * 0.02))); // SIM ONLY: fixed simulation period
    gyro.updateInputs(gyroIn);
    var pos = new SwerveModulePosition[4];
    var measured = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      pos[i] = modules[i].position();
      measured[i] = modules[i].velocity();
    }
    var pose =
        estimator.updateWithTime(edu.wpi.first.wpilibj.Timer.getFPGATimestamp(), gyroIn.yaw, pos);
    field.setRobotPose(pose);
    Logger.recordOutput("Drive/Pose", pose);
    Logger.recordOutput("Drive/GyroYaw", gyroIn.yaw);
    Logger.recordOutput("Drive/MeasuredModuleVelocities", measured);
    Logger.recordOutput("Drive/DesiredModuleVelocities", desired);
    Logger.recordOutput("Drive/ModulePositions", pos);
    Logger.recordOutput("Drive/CommandRobotRelative", command);
    Logger.recordOutput("Drive/MeasuredChassisSpeeds", kin.toChassisSpeeds(measured));
  }

  public void resetPose(Pose2d pose) {
    gyro.setYaw(pose.getRotation());
    gyro.updateInputs(gyroIn);
    estimator.resetPosition(gyroIn.yaw, positions(), pose);
  }

  public Pose2d getPose() {
    return estimator.getEstimatedPosition();
  }

  public Rotation2d getHeading() {
    return gyroIn.yaw;
  }

  /** Returns measured robot-relative chassis speeds for autonomous path following. */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    SwerveModuleState[] measured = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) measured[i] = modules[i].velocity();
    return kin.toChassisSpeeds(measured);
  }

  public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
    estimator.addVisionMeasurement(pose, timestamp, stdDevs);
  }

  public double getAngularVelocityRadiansPerSec() {
    return gyroIn.yawVelocityRadPerSec;
  }

  void setPathPlannerTrajectory(java.util.List<Pose2d> poses) {
    field.getObject("PathPlanner Trajectory").setPoses(poses);
  }

  SwerveModuleState[] desiredStates() {
    return desired.clone();
  }
}
