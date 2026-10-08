package frc.robot.Subsystems.drive.Motor.module;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.function.DoubleConsumer;

/** TODO HARDWARE: assign eight Spark MAX CAN IDs before constructing real module IO. */
public final class ModuleIOSpark implements ModuleIO {
  /** TODO MEASURE: set wheel radius plus drive and turn reductions. */
  public record Configuration(
      double drivePositionMetersPerMotorRotation,
      double driveVelocityMetersPerSecPerRpm,
      double turnPositionRadiansPerMotorRotation,
      double turnVelocityRadPerSecPerRpm) {
    public SparkMaxConfig driveConfig() {
      var config = new SparkMaxConfig();
      config.idleMode(IdleMode.kBrake); // TODO TUNE: real drive brake/coast behavior
      config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
      return config;
    }

    public SparkMaxConfig turnConfig() {
      var config = new SparkMaxConfig();
      config.idleMode(IdleMode.kBrake); // TODO TUNE: real turn brake/coast behavior
      config
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .positionWrappingEnabled(true);
      return config;
    }
  }

  public ModuleIOSpark() {
    throw new IllegalStateException(
        "Phase 1: Spark MAX CAN IDs are not configured"); // TODO HARDWARE: IDs
  }

  /** Establishes the relative turn reference once during real-module initialization. */
  static void initializeSteeringReference(
      SteeringReferenceMode mode, DoubleConsumer setTurnEncoderPosition) {
    if (mode == SteeringReferenceMode.MANUAL_ZERO)
      setTurnEncoderPosition.accept(0.0); // TODO CALIBRATE: BR zero
  }

  // Phase 2 will construct SparkMax(MotorType.kBrushless), use getEncoder() and
  // getClosedLoopController(), configure Configuration above, then command velocity/position.
}
