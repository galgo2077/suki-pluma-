package frc.robot.Subsystems.vision;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.Set;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.util.Units;
import org.wpilib.networktables.DoubleArrayPublisher;
import org.wpilib.networktables.DoubleArraySubscriber;
import org.wpilib.networktables.DoubleSubscriber;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.TimestampedDoubleArray;
import org.wpilib.system.RobotController;

/** Limelight 2 AprilTag adapter using the installed 2027 NetworkTables API. */
public final class VisionIOLimelight implements VisionIO {
  private static final long CONNECTION_TIMEOUT_MICROS =
      250_000; // TODO TUNE: Limelight disconnect timeout
  private final Supplier<Rotation2d> headingSupplier;
  private final DoubleArrayPublisher orientationPublisher;
  private final DoubleSubscriber heartbeatSubscriber;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;

  public VisionIOLimelight(String hostname, Supplier<Rotation2d> headingSupplier) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(hostname);
    this.headingSupplier = headingSupplier;
    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    heartbeatSubscriber = table.getDoubleTopic("hb").subscribe(0.0);
    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[0]);
    megatag2Subscriber = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[0]);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    Rotation2d heading = headingSupplier.get();
    orientationPublisher.accept(new double[] {heading.getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
    NetworkTableInstance.getDefault().flush();

    inputs.connected =
        RobotController.getTime() - heartbeatSubscriber.getLastChange() < CONNECTION_TIMEOUT_MICROS;
    Set<Integer> tagIds = new LinkedHashSet<>();
    var observations = new ArrayList<PoseObservation>();
    addObservations(
        megatag1Subscriber.readQueue(), PoseObservationType.MEGATAG_1, tagIds, observations);
    addObservations(
        megatag2Subscriber.readQueue(), PoseObservationType.MEGATAG_2, tagIds, observations);
    inputs.tagIds = tagIds.stream().mapToInt(Integer::intValue).toArray();
    inputs.poseObservations = observations.toArray(new PoseObservation[0]);
  }

  private void addObservations(
      TimestampedDoubleArray[] samples,
      PoseObservationType type,
      Set<Integer> tagIds,
      ArrayList<PoseObservation> observations) {
    for (var sample : samples) {
      double[] raw = sample.value;
      if (raw.length < 11) continue;
      for (int index = 11; index + 6 < raw.length; index += 7) tagIds.add((int) raw[index]);
      observations.add(
          new PoseObservation(
              sample.timestamp * 1.0e-6 - raw[6] * 1.0e-3,
              pose(raw),
              type == PoseObservationType.MEGATAG_1 && raw.length >= 18 ? raw[17] : 0.0,
              (int) raw[7],
              raw[9],
              type));
    }
  }

  private static Pose3d pose(double[] raw) {
    return new Pose3d(
        raw[0],
        raw[1],
        raw[2],
        new Rotation3d(
            Units.degreesToRadians(raw[3]),
            Units.degreesToRadians(raw[4]),
            Units.degreesToRadians(raw[5])));
  }
}
