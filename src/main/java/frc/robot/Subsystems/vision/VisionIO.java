package frc.robot.Subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

/** AprilTag-only localization camera interface. */
public interface VisionIO {
  @AutoLog
  class VisionIOInputs {
    public boolean connected;
    public int[] tagIds = new int[0];
    public PoseObservation[] poseObservations = new PoseObservation[0];
  }

  record PoseObservation(
      double timestamp, Pose3d pose, double ambiguity, int tagCount, double averageTagDistance) {}

  default void updateInputs(VisionIOInputs inputs) {}
}
