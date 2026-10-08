package frc.robot.Subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.Constants.SystemConstants.OdometryMode;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.system.Timer;

class VisionTest {
  private static VisionIO.PoseObservation observation(
      double timestamp,
      Pose3d pose,
      int tagCount,
      double ambiguity,
      VisionIO.PoseObservationType type) {
    return new VisionIO.PoseObservation(timestamp, pose, ambiguity, tagCount, 2.0, type);
  }

  @Test
  void limelightModeFusesValidObservationIntoTheSameDriveEstimator() {
    Drive drive = new Drive();
    double timestamp = Timer.getTimestamp();
    drive.periodic();
    double before = drive.getPose().getX();
    VisionIOSim io = new VisionIOSim();
    io.addObservation(
        observation(
            timestamp,
            new Pose3d(2.0, 1.0, 0.0, new Rotation3d()),
            2,
            0.0,
            VisionIO.PoseObservationType.MEGATAG_1));
    new Vision(drive::addVisionMeasurement, () -> 0.0, OdometryMode.ODOMETRY_LIMELIGHT, io)
        .periodic();
    assertTrue(drive.getPose().getX() > before);
    assertTrue(drive.getPose().getX() < 2.0);
  }

  @Test
  void odometryOnlyDoesNotSubmitVision() {
    Drive drive = new Drive();
    double timestamp = Timer.getTimestamp();
    drive.periodic();
    double before = drive.getPose().getX();
    VisionIOSim io = new VisionIOSim();
    io.addObservation(
        observation(
            timestamp,
            new Pose3d(2.0, 1.0, 0.0, new Rotation3d()),
            2,
            0.0,
            VisionIO.PoseObservationType.MEGATAG_1));
    new Vision(drive::addVisionMeasurement, () -> 0.0, OdometryMode.ODOMETRY_ONLY, io).periodic();
    assertEquals(before, drive.getPose().getX());
  }

  @Test
  void disconnectFallsBackAndReconnectResumes() {
    Drive drive = new Drive();
    VisionIOSim io = new VisionIOSim();
    Vision vision =
        new Vision(drive::addVisionMeasurement, () -> 0.0, OdometryMode.ODOMETRY_LIMELIGHT, io);
    io.setConnected(false);
    drive.driveRobotRelative(new ChassisVelocities(1.0, 0.0, 0.0));
    drive.periodic();
    drive.periodic();
    double odometryX = drive.getPose().getX();
    vision.periodic();
    assertTrue(odometryX > 0.0);
    assertTrue(Double.isFinite(drive.getPose().getX()));

    double timestamp = Timer.getTimestamp();
    drive.periodic();
    io.setConnected(true);
    io.addObservation(
        observation(
            timestamp,
            new Pose3d(3.0, 0.0, 0.0, new Rotation3d()),
            2,
            0.0,
            VisionIO.PoseObservationType.MEGATAG_1));
    vision.periodic();
    assertTrue(drive.getPose().getX() > odometryX);
  }

  @Test
  void invalidObservationsAreRejected() {
    AtomicInteger submissions = new AtomicInteger();
    VisionIOSim io = new VisionIOSim();
    double timestamp = Timer.getTimestamp();
    io.addObservation(
        observation(timestamp, new Pose3d(), 0, 0.0, VisionIO.PoseObservationType.MEGATAG_1));
    io.addObservation(
        observation(
            timestamp,
            new Pose3d(-1.0, 0.0, 0.0, new Rotation3d()),
            2,
            0.0,
            VisionIO.PoseObservationType.MEGATAG_1));
    io.addObservation(
        observation(
            timestamp,
            new Pose3d(1.0, 1.0, 2.0, new Rotation3d()),
            2,
            0.0,
            VisionIO.PoseObservationType.MEGATAG_1));
    io.addObservation(
        observation(
            timestamp,
            new Pose3d(1.0, 1.0, 0.0, new Rotation3d()),
            1,
            VisionConstants.MAX_SINGLE_TAG_AMBIGUITY + 0.01,
            VisionIO.PoseObservationType.MEGATAG_1));
    io.addObservation(
        observation(
            Double.NaN,
            new Pose3d(Double.NaN, 1.0, 0.0, new Rotation3d()),
            2,
            0.0,
            VisionIO.PoseObservationType.MEGATAG_1));
    new Vision(
            (pose, sampleTime, stdDevs) -> submissions.incrementAndGet(),
            () -> 0.0,
            OdometryMode.ODOMETRY_LIMELIGHT,
            io)
        .periodic();
    assertEquals(0, submissions.get());
  }

  @Test
  void megatagTypesGetDistinctConfiguredConfidence() {
    List<Double> angularStdDevs = new ArrayList<>();
    VisionIOSim io = new VisionIOSim();
    double timestamp = Timer.getTimestamp();
    io.addObservation(
        observation(
            timestamp,
            new Pose3d(1.0, 1.0, 0.0, new Rotation3d()),
            2,
            0.0,
            VisionIO.PoseObservationType.MEGATAG_1));
    io.addObservation(
        observation(
            timestamp,
            new Pose3d(1.0, 1.0, 0.0, new Rotation3d()),
            2,
            0.0,
            VisionIO.PoseObservationType.MEGATAG_2));
    new Vision(
            (pose, sampleTime, stdDevs) -> angularStdDevs.add(stdDevs.get(2, 0)),
            () -> 0.0,
            OdometryMode.ODOMETRY_LIMELIGHT,
            io)
        .periodic();
    assertEquals(2, angularStdDevs.size());
    assertTrue(Double.isFinite(angularStdDevs.get(0)));
    assertEquals(Double.POSITIVE_INFINITY, angularStdDevs.get(1));
  }
}
