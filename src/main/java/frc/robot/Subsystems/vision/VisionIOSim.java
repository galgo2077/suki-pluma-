package frc.robot.Subsystems.vision;

import java.util.ArrayDeque;
import java.util.Arrays;

/** Deterministic AprilTag observation source for localization tests and simulation. */
public final class VisionIOSim implements VisionIO {
  private boolean connected = true;
  private int[] tagIds = new int[0];
  private final ArrayDeque<PoseObservation> observations = new ArrayDeque<>();

  public void setConnected(boolean connected) {
    this.connected = connected;
  }

  public void setTagIds(int... tagIds) {
    this.tagIds = tagIds.clone();
  }

  public void addObservation(PoseObservation observation) {
    observations.add(observation);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = connected;
    inputs.tagIds = Arrays.copyOf(tagIds, tagIds.length);
    inputs.poseObservations =
        connected ? observations.toArray(new PoseObservation[0]) : new PoseObservation[0];
    observations.clear();
  }
}
