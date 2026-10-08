package frc.robot.Subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Pose3d;

/** AprilTag-only localization camera interface. */
public interface VisionIO {
  @AutoLog
  class VisionIOInputs {
    public boolean connected;
    public int[] tagIds = new int[0];
    public PoseObservation[] poseObservations = new PoseObservation[0];
  }

  record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {}

  enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2
  }

  default void updateInputs(VisionIOInputs inputs) {}
}
