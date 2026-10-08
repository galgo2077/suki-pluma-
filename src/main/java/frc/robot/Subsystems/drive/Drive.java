package frc.robot.Subsystems.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SystemConstants;
import frc.robot.Subsystems.Odometry.Gyro.GyroIO;
import frc.robot.Subsystems.Odometry.Gyro.GyroIOInputsAutoLogged;
import frc.robot.Subsystems.Odometry.Gyro.GyroIOPigeon2;
import frc.robot.Subsystems.Odometry.Gyro.GyroIOSim;
import frc.robot.Subsystems.drive.Encoders.AbsoluteEncoderIO;
import frc.robot.Subsystems.drive.Encoders.AbsoluteEncoderIOCANcoder;
import frc.robot.Subsystems.drive.Encoders.AbsoluteEncoderIODutyCycle;
import frc.robot.Subsystems.drive.Motor.module.Module;
import frc.robot.Subsystems.drive.Motor.module.ModuleIO;
import frc.robot.Subsystems.drive.Motor.module.ModuleIOSim;
import frc.robot.Subsystems.drive.Motor.module.ModuleIOSpark;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private final Module[] modules = {
    new Module("FL", createModuleIO(0)),
    new Module("FR", createModuleIO(1)),
    new Module("BL", createModuleIO(2)),
    new Module("BR", createModuleIO(3))
  };
  private final GyroIO gyro = createGyroIO();
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

  private static ModuleIO createModuleIO(int index) {
    if (SystemConstants.currentMode == SystemConstants.Mode.SIM) return new ModuleIOSim();
    if (SystemConstants.currentMode != SystemConstants.Mode.REAL
        || !DriveConstants.realHardwareConfigurationValid()) return new ModuleIO() {};
    var hardware = DriveConstants.MODULE_HARDWARE[index];
    AbsoluteEncoderIO absoluteEncoder =
        index == 3
            ? new AbsoluteEncoderIODutyCycle(hardware.throughBoreDio())
            : new AbsoluteEncoderIOCANcoder(new CANcoder(hardware.canCoderCanId()));
    return new ModuleIOSpark(
        hardware,
        new ModuleIOSpark.Configuration(
            DriveConstants.DRIVE_POSITION_METERS_PER_MOTOR_ROTATION,
            DriveConstants.DRIVE_VELOCITY_METERS_PER_SECOND_PER_RPM,
            DriveConstants.TURN_POSITION_RADIANS_PER_MOTOR_ROTATION,
            DriveConstants.TURN_VELOCITY_RADIANS_PER_SECOND_PER_RPM,
            DriveConstants.DRIVE_KP,
            DriveConstants.TURN_KP),
        absoluteEncoder);
  }

  private static GyroIO createGyroIO() {
    if (SystemConstants.currentMode == SystemConstants.Mode.SIM) return new GyroIOSim();
    if (SystemConstants.currentMode == SystemConstants.Mode.REAL
        && DriveConstants.realHardwareConfigurationValid())
      return new GyroIOPigeon2(new Pigeon2(DriveConstants.PIGEON_CAN_ID));
    return new GyroIO() {};
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

  public void stop() {
    command = new ChassisSpeeds();
    desired =
        new SwerveModuleState[] {
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState()
        };
    for (Module module : modules) module.stop();
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) stop();
    for (int i = 0; i < 4; i++) {
      modules[i].periodic();
    }
    if (gyro instanceof GyroIOSim simGyro)
      simGyro.setYaw(
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
    if (gyro instanceof GyroIOSim simGyro) simGyro.setYaw(pose.getRotation());
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
