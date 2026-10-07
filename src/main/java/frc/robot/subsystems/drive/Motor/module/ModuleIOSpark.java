package frc.robot.subsystems.drive.Motor.module;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.function.DoubleConsumer;

/**
 * Future real NEO v1.1 / Spark MAX IO. Phase 2 supplies CAN IDs and establishes the steering
 * relative encoder reference from the absolute encoder; Phase 1 deliberately constructs nothing.
 */
public final class ModuleIOSpark implements ModuleIO {
  /** Unit conversion and closed-loop configuration shared by every planned NEO/Spark MAX module. */
  public record Configuration(
      double drivePositionMetersPerMotorRotation,
      double driveVelocityMetersPerSecPerRpm,
      double turnPositionRadiansPerMotorRotation,
      double turnVelocityRadPerSecPerRpm) {
    public SparkMaxConfig driveConfig() {
      var config = new SparkMaxConfig();
      config.idleMode(IdleMode.kBrake);
      config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
      return config;
    }

    public SparkMaxConfig turnConfig() {
      var config = new SparkMaxConfig();
      config.idleMode(IdleMode.kBrake);
      config
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .positionWrappingEnabled(true);
      return config;
    }
  }

  public ModuleIOSpark() {
    throw new IllegalStateException("Phase 1: Spark MAX CAN IDs are not configured");
  }

  /** Establishes the relative turn reference once during real-module initialization. */
  static void initializeSteeringReference(
      SteeringReferenceMode mode, DoubleConsumer setTurnEncoderPosition) {
    if (mode == SteeringReferenceMode.MANUAL_ZERO) setTurnEncoderPosition.accept(0.0);
  }

  // Phase 2 will construct SparkMax(MotorType.kBrushless), use getEncoder() and
  // getClosedLoopController(), configure Configuration above, then command velocity/position.
}
