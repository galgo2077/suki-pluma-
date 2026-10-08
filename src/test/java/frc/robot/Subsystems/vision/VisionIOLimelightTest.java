package frc.robot.Subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.networktables.NetworkTableInstance;
import org.junit.jupiter.api.Test;

class VisionIOLimelightTest {
  @Test
  void readsStandardAprilTagPoseOnly() {
    var instance = NetworkTableInstance.getDefault();
    instance.startLocal();
    try {
      var io = new VisionIOLimelight("limelight");
      instance
          .getTable("limelight")
          .getDoubleArrayTopic("botpose_wpiblue")
          .publish()
          .set(new double[] {1.0, 2.0, 0.0, 0.0, 0.0, 90.0, 0.0, 2.0, 0.0, 1.5, 0.0});
      instance
          .getTable("limelight")
          .getDoubleArrayTopic("botpose_orb_wpiblue")
          .publish()
          .set(new double[] {9.0, 9.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 1.5, 0.0});
      instance.flush();
      var inputs = new VisionIO.VisionIOInputs();
      io.updateInputs(inputs);
      assertAll(
          () -> assertEquals(1, inputs.poseObservations.length),
          () -> assertEquals(1.0, inputs.poseObservations[0].pose().getX(), 1e-9),
          () -> assertEquals(2.0, inputs.poseObservations[0].pose().getY(), 1e-9),
          () -> assertEquals(1.5, inputs.poseObservations[0].averageTagDistance(), 1e-9));
    } finally {
      instance.stopLocal();
    }
  }
}
