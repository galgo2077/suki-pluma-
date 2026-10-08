package frc.robot.Subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.Set;

/** Limelight 2 AprilTag adapter using NetworkTables. */
public final class VisionIOLimelight implements VisionIO {
  private static final long CONNECTION_TIMEOUT_MICROS =
      250_000; // TODO TUNE: Limelight disconnect timeout
  private final DoubleSubscriber heartbeatSubscriber;
  private final DoubleArraySubscriber aprilTagSubscriber;

  public VisionIOLimelight(String hostname) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(hostname);
    heartbeatSubscriber = table.getDoubleTopic("hb").subscribe(0.0);
    aprilTagSubscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[0]);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected =
        RobotController.getTime() - heartbeatSubscriber.getLastChange() < CONNECTION_TIMEOUT_MICROS;
    Set<Integer> tagIds = new LinkedHashSet<>();
    var observations = new ArrayList<PoseObservation>();
    addObservations(aprilTagSubscriber.readQueue(), tagIds, observations);
    inputs.tagIds = tagIds.stream().mapToInt(Integer::intValue).toArray();
    inputs.poseObservations = observations.toArray(new PoseObservation[0]);
  }

  private void addObservations(
      TimestampedDoubleArray[] samples,
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
              raw.length >= 18 ? raw[17] : 0.0,
              (int) raw[7],
              raw[9]));
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
