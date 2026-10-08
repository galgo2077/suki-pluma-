package frc.robot.Subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.SystemConstants;
import frc.robot.Constants.SystemConstants.OdometryMode;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class VisionValidationTest {
  private record Submission(
      Pose2d pose, double timestamp, double linearStdDev, double angularStdDev) {}

  private static VisionIO.PoseObservation observation(
      double timestamp,
      double x,
      double y,
      double z,
      int tagCount,
      double ambiguity,
      double distance) {
    return new VisionIO.PoseObservation(
        timestamp, new Pose3d(x, y, z, new Rotation3d()), ambiguity, tagCount, distance);
  }

  private static Vision vision(VisionIOSim io, OdometryMode mode, List<Submission> submissions) {
    return new Vision(
        (pose, timestamp, stdDevs) ->
            submissions.add(new Submission(pose, timestamp, stdDevs.get(0, 0), stdDevs.get(2, 0))),
        mode,
        io);
  }

  @Test
  void odometryOnlyBaselineMovesAndIgnoresVision() {
    Drive drive = new Drive();
    drive.resetPose(new Pose2d());
    drive.driveRobotRelative(new ChassisSpeeds(1.0, 0.5, 0.0));
    drive.periodic();
    drive.periodic();
    Pose2d beforeVision = drive.getPose();
    assertAll(
        () -> assertTrue(beforeVision.getX() > 0.0),
        () -> assertTrue(Double.isFinite(beforeVision.getX())),
        () -> assertTrue(Double.isFinite(beforeVision.getY())),
        () -> assertTrue(Double.isFinite(beforeVision.getRotation().getRadians())));

    VisionIOSim io = new VisionIOSim();
    io.addObservation(observation(Timer.getTimestamp(), 4.0, 1.0, 0.0, 2, 0.0, 1.0));
    new Vision(drive::addVisionMeasurement, OdometryMode.ODOMETRY_ONLY, io).periodic();
    assertEquals(beforeVision, drive.getPose());
  }

  @Test
  void fusionCorrectsWithoutHardReset() {
    Drive drive = new Drive();
    drive.resetPose(new Pose2d(2.0, 0.0, new Rotation2d()));
    double timestamp = Timer.getTimestamp();
    drive.periodic();
    Pose2d visionPose = new Pose2d(1.7, 0.0, new Rotation2d());
    double distanceBefore =
        drive.getPose().getTranslation().getDistance(visionPose.getTranslation());
    VisionIOSim io = new VisionIOSim();
    io.addObservation(observation(timestamp, 1.7, 0.0, 0.0, 2, 0.0, 1.0));
    new Vision(drive::addVisionMeasurement, OdometryMode.ODOMETRY_LIMELIGHT, io).periodic();
    double distanceAfter =
        drive.getPose().getTranslation().getDistance(visionPose.getTranslation());
    assertTrue(distanceAfter < distanceBefore);
    assertNotEquals(visionPose, drive.getPose());
  }

  @Test
  void disconnectedAndRejectedVisionNeverFreezeOdometryOrMutateMode() {
    Drive drive = new Drive();
    VisionIOSim io = new VisionIOSim();
    List<Submission> submissions = new ArrayList<>();
    Vision vision = vision(io, OdometryMode.ODOMETRY_LIMELIGHT, submissions);
    OdometryMode configuredMode = SystemConstants.ODOMETRY_MODE;
    io.addObservation(observation(Timer.getTimestamp(), 1.0, 0.0, 0.0, 2, 0.0, 1.0));
    vision.periodic();
    assertEquals(1, submissions.size());
    io.setConnected(false);
    drive.driveRobotRelative(new ChassisSpeeds(1.0, 0.0, 0.0));
    for (int i = 0; i < 4; i++) {
      io.addObservation(observation(Timer.getTimestamp(), -1.0, 0.0, 0.0, 0, 1.0, 1.0));
      drive.periodic();
      vision.periodic();
    }
    double offlineX = drive.getPose().getX();
    assertAll(
        () -> assertTrue(offlineX > 0.0),
        () -> assertTrue(Double.isFinite(offlineX)),
        () -> assertEquals(1, submissions.size()),
        () -> assertEquals(configuredMode, SystemConstants.ODOMETRY_MODE));

    double timestamp = Timer.getTimestamp();
    drive.periodic();
    io.setConnected(true);
    io.addObservation(observation(timestamp, 3.0, 0.0, 0.0, 2, 0.0, 1.0));
    vision.periodic();
    assertAll(
        () -> assertEquals(2, submissions.size()),
        () -> assertTrue(drive.getPose().getX() > offlineX),
        () -> assertEquals(configuredMode, SystemConstants.ODOMETRY_MODE));
  }

  @Test
  void filterBoundariesAcceptOnlyEligibleObservations() {
    VisionIOSim io = new VisionIOSim();
    List<Submission> submissions = new ArrayList<>();
    double now = Timer.getTimestamp();
    double maxX = VisionConstants.APRILTAG_LAYOUT.getFieldLength();
    double maxY = VisionConstants.APRILTAG_LAYOUT.getFieldWidth();
    io.addObservation(observation(now, -0.01, 1.0, 0.0, 2, 0.0, 1.0));
    io.addObservation(observation(now, maxX + 0.01, 1.0, 0.0, 2, 0.0, 1.0));
    io.addObservation(observation(now, 1.0, -0.01, 0.0, 2, 0.0, 1.0));
    io.addObservation(observation(now, 1.0, maxY + 0.01, 0.0, 2, 0.0, 1.0));
    io.addObservation(
        observation(now, 1.0, 1.0, VisionConstants.MAX_Z_ERROR_METERS + 0.01, 2, 0.0, 1.0));
    io.addObservation(observation(now, 0.01, 0.01, 0.0, 1, 0.0, 1.0));
    vision(io, OdometryMode.ODOMETRY_LIMELIGHT, submissions).periodic();
    assertEquals(1, submissions.size());
    assertEquals(0.01, submissions.get(0).pose().getX(), 1e-9);
    assertEquals(0.01, submissions.get(0).pose().getY(), 1e-9);
  }

  @Test
  void rejectsNonFiniteAndInvalidTimestampValues() {
    VisionIOSim io = new VisionIOSim();
    List<Submission> submissions = new ArrayList<>();
    double now = Timer.getTimestamp();
    for (double value :
        new double[] {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
      io.addObservation(observation(now, value, 1.0, 0.0, 2, 0.0, 1.0));
      io.addObservation(observation(value, 1.0, 1.0, 0.0, 2, 0.0, 1.0));
    }
    io.addObservation(observation(0.0, 1.0, 1.0, 0.0, 2, 0.0, 1.0));
    io.addObservation(observation(-1.0, 1.0, 1.0, 0.0, 2, 0.0, 1.0));
    vision(io, OdometryMode.ODOMETRY_LIMELIGHT, submissions).periodic();
    assertTrue(submissions.isEmpty());
  }

  @Test
  void aprilTagObservationsRemainIndependentAndUseQualityConfidence() {
    VisionIOSim io = new VisionIOSim();
    List<Submission> submissions = new ArrayList<>();
    double now = Timer.getTimestamp();
    io.addObservation(observation(now, 1.0, 2.0, 0.0, 1, 0.1, 1.0));
    io.addObservation(observation(now + 0.001, 3.0, 2.0, 0.0, 2, 0.0, 4.0));
    io.addObservation(observation(now + 0.002, 4.0, 2.0, 0.0, 2, 0.9, 1.0));
    vision(io, OdometryMode.ODOMETRY_LIMELIGHT, submissions).periodic();
    assertAll(
        () -> assertEquals(3, submissions.size()),
        () -> assertEquals(1.0, submissions.get(0).pose().getX(), 1e-9),
        () -> assertEquals(3.0, submissions.get(1).pose().getX(), 1e-9),
        () -> assertEquals(4.0, submissions.get(2).pose().getX(), 1e-9),
        () -> assertTrue(submissions.get(1).linearStdDev() > submissions.get(0).linearStdDev()),
        () -> assertTrue(submissions.get(2).linearStdDev() < submissions.get(0).linearStdDev()),
        () -> assertTrue(Double.isFinite(submissions.get(1).angularStdDev())));
  }

  @Test
  void visionIoSimIsDeterministicAndPreservesCoordinates() {
    VisionIOSim io = new VisionIOSim();
    io.setTagIds(1, 2);
    VisionIO.PoseObservation first = observation(1.0, 1.25, 2.5, 0.0, 1, 0.1, 1.0);
    VisionIO.PoseObservation second = observation(2.0, 3.75, 4.5, 0.0, 2, 0.0, 2.0);
    io.addObservation(first);
    io.addObservation(second);
    VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();
    io.updateInputs(inputs);
    assertAll(
        () -> assertTrue(inputs.connected),
        () -> assertArrayEquals(new int[] {1, 2}, inputs.tagIds),
        () ->
            assertArrayEquals(
                new VisionIO.PoseObservation[] {first, second}, inputs.poseObservations),
        () -> assertEquals(1.25, inputs.poseObservations[0].pose().getX(), 1e-9),
        () -> assertEquals(2.5, inputs.poseObservations[0].pose().getY(), 1e-9));
    io.updateInputs(inputs);
    assertEquals(0, inputs.poseObservations.length);
  }

  @Test
  void headingWrapAndLoggingAreSafe() {
    Drive drive = new Drive();
    drive.resetPose(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(359.0)));
    drive.periodic();
    drive.resetPose(new Pose2d(0.0, 0.0, new Rotation2d()));
    drive.periodic();
    drive.resetPose(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(1.0)));
    drive.periodic();
    assertTrue(Math.abs(drive.getHeading().minus(Rotation2d.fromDegrees(1.0)).getRadians()) < 1e-9);

    VisionIOSim io = new VisionIOSim();
    assertDoesNotThrow(
        () -> vision(io, OdometryMode.ODOMETRY_LIMELIGHT, new ArrayList<>()).periodic());
  }
}
