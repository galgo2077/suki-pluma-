package frc.robot.Subsystems.vision;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SystemConstants.OdometryMode;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

/** Filters AprilTag observations and submits valid samples to Drive's single estimator. */
public final class Vision extends SubsystemBase {
  @FunctionalInterface
  public interface VisionConsumer {
    void accept(Pose2d pose, double timestamp, Matrix<N3, N1> standardDeviations);
  }

  private final VisionConsumer consumer;
  private final OdometryMode odometryMode;
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  public Vision(VisionConsumer consumer, OdometryMode odometryMode, VisionIO io) {
    this.consumer = consumer;
    this.odometryMode = odometryMode;
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);
    Logger.recordOutput("Vision/Connected", inputs.connected);
    Logger.recordOutput("Vision/Mode", odometryMode.toString());
    Logger.recordOutput("Vision/TagIds", inputs.tagIds);
    Logger.recordOutput("Vision/ObservationCount", inputs.poseObservations.length);

    var accepted = new ArrayList<Pose3d>();
    var rejected = new ArrayList<Pose3d>();
    var tagPoses = new ArrayList<Pose3d>();
    for (int tagId : inputs.tagIds) APRILTAG_LAYOUT.getTagPose(tagId).ifPresent(tagPoses::add);

    double linearStdDev = Double.NaN;
    double angularStdDev = Double.NaN;
    for (var observation : inputs.poseObservations) {
      if (rejected(observation)) {
        rejected.add(observation.pose());
        continue;
      }
      accepted.add(observation.pose());
      double factor =
          Math.pow(observation.averageTagDistance(), 2.0)
              / observation.tagCount(); // TODO TUNE: distance and tag-count confidence scaling
      linearStdDev = LINEAR_STD_DEV_BASELINE * factor;
      angularStdDev = ANGULAR_STD_DEV_BASELINE * factor;
      if (odometryMode == OdometryMode.ODOMETRY_LIMELIGHT)
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
    }

    Logger.recordOutput("Vision/TagPoses", tagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/AprilTag/RobotPose", accepted.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/AcceptedPoses", accepted.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/RejectedPoses", rejected.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/AcceptedCount", accepted.size());
    Logger.recordOutput("Vision/RejectedCount", rejected.size());
    Logger.recordOutput("Vision/LinearStdDev", linearStdDev);
    Logger.recordOutput("Vision/AngularStdDev", angularStdDev);
  }

  private boolean rejected(VisionIO.PoseObservation observation) {
    Pose3d pose = observation.pose();
    return !inputs.connected
        || observation.tagCount() == 0
        || (observation.tagCount() == 1 && observation.ambiguity() > MAX_SINGLE_TAG_AMBIGUITY)
        || !finite(observation.timestamp())
        || observation.timestamp() <= 0.0
        || !finite(pose.getX())
        || !finite(pose.getY())
        || !finite(pose.getZ())
        || !finite(pose.getRotation().getX())
        || !finite(pose.getRotation().getY())
        || !finite(pose.getRotation().getZ())
        || Math.abs(pose.getZ()) > MAX_Z_ERROR_METERS
        || pose.getX() < 0.0 // TODO TUNE: field X margin
        || pose.getX() > APRILTAG_LAYOUT.getFieldLength()
        || pose.getY() < 0.0 // TODO TUNE: field Y margin
        || pose.getY() > APRILTAG_LAYOUT.getFieldWidth();
  }

  private static boolean finite(double value) {
    return Double.isFinite(value);
  }
}
