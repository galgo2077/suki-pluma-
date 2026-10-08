package frc.robot.Subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.SystemConstants.OdometryMode;
import frc.robot.Subsystems.vision.Vision;
import frc.robot.Subsystems.vision.VisionIO;
import frc.robot.Subsystems.vision.VisionIOSim;
import org.junit.jupiter.api.Test;

class PoseEstimatorTest {
  @Test
  void aprilTagObservationCorrectsTheExistingPoseEstimator() {
    var drive = new Drive();
    drive.periodic();
    double before = drive.getPose().getX();
    var io = new VisionIOSim();
    io.addObservation(
        new VisionIO.PoseObservation(
            Timer.getTimestamp(), new Pose3d(2.0, 1.0, 0.0, new Rotation3d()), 0.0, 2, 1.0));

    new Vision(drive::addVisionMeasurement, OdometryMode.ODOMETRY_LIMELIGHT, io).periodic();

    assertTrue(drive.getPose().getX() > before);
  }
}
