package frc.robot.subsystems.drive;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.drive.Gyro.GyroIOSim;
import frc.robot.subsystems.drive.Motor.module.Module;
import frc.robot.subsystems.drive.Motor.module.ModuleIOSim;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.*;
import org.wpilib.math.kinematics.*;

public class Drive implements Mechanism {
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
      new SwerveDrivePoseEstimator(kin, Rotation2d.ZERO, positions(), new Pose2d());

  private ChassisVelocities command = new ChassisVelocities();
  private SwerveModuleVelocity[] desired = new SwerveModuleVelocity[4];

  public Drive() {
    for (int i = 0; i < 4; i++) desired[i] = new SwerveModuleVelocity();
  }

  private SwerveModulePosition[] positions() {
    SwerveModulePosition[] a = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) a[i] = modules[i].position();
    return a;
  }

  public void driveFieldRelative(double x, double y, double omega) {
    driveRobotRelative(new ChassisVelocities(x, y, omega).toRobotRelative(gyroIn.yaw));
    Logger.recordOutput("Drive/CommandFieldRelative", new ChassisVelocities(x, y, omega));
  }

  public void driveRobotRelative(ChassisVelocities speeds) {
    command = speeds;
    desired =
        SwerveDriveKinematics.desaturateWheelVelocities(
            kin.toSwerveModuleVelocities(speeds), DriveConstants.MAX_SPEED);
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(desired[i]);
    }
  }

  public void periodic() {
    for (int i = 0; i < 4; i++) {
      modules[i].periodic();
    }
    gyro.setYaw(gyroIn.yaw.plus(new Rotation2d(command.omega * 0.02)));
    gyro.updateInputs(gyroIn);
    var pos = new SwerveModulePosition[4];
    var measured = new SwerveModuleVelocity[4];
    for (int i = 0; i < 4; i++) {
      pos[i] = modules[i].position();
      measured[i] = modules[i].velocity();
    }
    var pose = estimator.updateWithTime(org.wpilib.system.Timer.getTimestamp(), gyroIn.yaw, pos);
    Logger.recordOutput("Drive/Pose", pose);
    Logger.recordOutput("Drive/GyroYaw", gyroIn.yaw);
    Logger.recordOutput("Drive/MeasuredModuleVelocities", measured);
    Logger.recordOutput("Drive/DesiredModuleVelocities", desired);
    Logger.recordOutput("Drive/ModulePositions", pos);
    Logger.recordOutput("Drive/CommandRobotRelative", command);
    Logger.recordOutput("Drive/MeasuredChassisVelocities", kin.toChassisVelocities(measured));
  }

  public void resetPose(Pose2d pose) {
    gyro.setYaw(pose.getRotation());
    gyro.updateInputs(gyroIn);
    estimator.resetPosition(gyroIn.yaw, positions(), pose);
  }

  public Pose2d pose() {
    return estimator.getEstimatedPosition();
  }

  Rotation2d heading() {
    return gyroIn.yaw;
  }

  SwerveModuleVelocity[] desiredStates() {
    return desired.clone();
  }
}
