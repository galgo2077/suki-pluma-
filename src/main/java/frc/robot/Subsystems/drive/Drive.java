package frc.robot.Subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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

  private final Field2d field = new Field2d();

  public Drive() {
    SmartDashboard.putData("Swerve Field", field);
  }

  private SwerveModulePosition[] positions() {
    SwerveModulePosition[] a = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) a[i] = modules[i].position();
    return a;
  }

  @Override
  public void periodic() {
    for (int i = 0; i < 4; i++) {
      modules[i].periodic();
    }
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
    Logger.recordOutput("Drive/ModulePositions", pos);
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

  public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
    estimator.addVisionMeasurement(pose, timestamp, stdDevs);
  }
}
