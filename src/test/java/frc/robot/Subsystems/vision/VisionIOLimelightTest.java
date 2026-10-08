package frc.robot.Subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.networktables.NetworkTableInstance;

class VisionIOLimelightTest {
  @Test
  void sendsEstimatorHeadingToMegaTag2() {
    var instance = NetworkTableInstance.getDefault();
    instance.startLocal();
    var orientation =
        instance
            .getTable("limelight")
            .getDoubleArrayTopic("robot_orientation_set")
            .subscribe(new double[0]);
    try {
      new VisionIOLimelight("limelight", () -> Rotation2d.fromDegrees(37.0))
          .updateInputs(new VisionIO.VisionIOInputs());
      assertEquals(37.0, orientation.get()[0], 1e-9);
    } finally {
      orientation.close();
      instance.stopLocal();
    }
  }
}
