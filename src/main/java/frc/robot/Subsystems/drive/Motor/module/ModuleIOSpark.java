package frc.robot.Subsystems.drive.Motor.module;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.drive.Encoders.AbsoluteEncoderIO;
import frc.robot.Subsystems.drive.Encoders.AbsoluteEncoderIOInputsAutoLogged;
import java.util.function.DoubleConsumer;

/** Modern REV implementation for one NEO drive/turn swerve module. */
public final class ModuleIOSpark implements ModuleIO {
  private final SparkMax drive;
  private final SparkMax turn;
  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnEncoder;
  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController turnController;
  private final AbsoluteEncoderIO absoluteEncoder;
  private final AbsoluteEncoderIOInputsAutoLogged absoluteInputs =
      new AbsoluteEncoderIOInputsAutoLogged();
  private final double absoluteOffsetRadians;

  public ModuleIOSpark(
      DriveConstants.ModuleHardware hardware,
      Configuration configuration,
      AbsoluteEncoderIO absoluteEncoder) {
    drive = new SparkMax(hardware.driveCanId(), MotorType.kBrushless);
    turn = new SparkMax(hardware.turnCanId(), MotorType.kBrushless);
    driveEncoder = drive.getEncoder();
    turnEncoder = turn.getEncoder();
    driveController = drive.getClosedLoopController();
    turnController = turn.getClosedLoopController();
    this.absoluteEncoder = absoluteEncoder;
    absoluteOffsetRadians =
        hardware.absoluteOffsetRadians() == null ? 0.0 : hardware.absoluteOffsetRadians();

    if (drive.configure(
                configuration.driveConfig(hardware),
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters)
            != REVLibError.kOk
        || turn.configure(
                configuration.turnConfig(hardware),
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters)
            != REVLibError.kOk)
      throw new IllegalStateException("Failed to configure swerve SPARK MAX");

    absoluteEncoder.updateInputs(absoluteInputs);
    if (!absoluteInputs.connected)
      throw new IllegalStateException("Swerve absolute encoder is not connected");
    turnEncoder.setPosition(relativeAngle(absoluteInputs.absolutePosition).getRadians());
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    absoluteEncoder.updateInputs(absoluteInputs);
    inputs.drivePositionMeters = driveEncoder.getPosition();
    inputs.driveVelocityMetersPerSec = driveEncoder.getVelocity();
    inputs.turnPosition = Rotation2d.fromRadians(turnEncoder.getPosition());
    inputs.turnVelocityRadPerSec = turnEncoder.getVelocity();
    inputs.absoluteTurnPosition = relativeAngle(absoluteInputs.absolutePosition);
    inputs.driveCurrentAmps = drive.getOutputCurrent();
    inputs.turnCurrentAmps = turn.getOutputCurrent();
    inputs.absoluteEncoderConnected = absoluteInputs.connected;
    inputs.fault = drive.hasActiveFault() || turn.hasActiveFault();
    inputs.connected = absoluteInputs.connected && !inputs.fault;
  }

  @Override
  public void setDesired(double metersPerSec, Rotation2d angle) {
    driveController.setSetpoint(metersPerSec, ControlType.kVelocity);
    turnController.setSetpoint(angle.getRadians(), ControlType.kPosition);
  }

  @Override
  public void stop() {
    drive.stopMotor();
    turn.stopMotor();
  }

  private Rotation2d relativeAngle(Rotation2d absoluteAngle) {
    return Rotation2d.fromRadians(
        MathUtil.angleModulus(absoluteAngle.getRadians() - absoluteOffsetRadians));
  }

  public record Configuration(
      double drivePositionMetersPerMotorRotation,
      double driveVelocityMetersPerSecPerRpm,
      double turnPositionRadiansPerMotorRotation,
      double turnVelocityRadPerSecPerRpm,
      double driveKp,
      double turnKp) {
    SparkMaxConfig driveConfig(DriveConstants.ModuleHardware hardware) {
      var config = new SparkMaxConfig();
      config.inverted(hardware.driveInverted()).idleMode(IdleMode.kBrake);
      if (hardware.driveCurrentLimitAmps() != null)
        config.smartCurrentLimit(hardware.driveCurrentLimitAmps());
      if (Double.isFinite(drivePositionMetersPerMotorRotation))
        config.encoder.positionConversionFactor(drivePositionMetersPerMotorRotation);
      if (Double.isFinite(driveVelocityMetersPerSecPerRpm))
        config.encoder.velocityConversionFactor(driveVelocityMetersPerSecPerRpm);
      config.closedLoop.feedbackSensor(com.revrobotics.spark.FeedbackSensor.kPrimaryEncoder);
      if (Double.isFinite(driveKp)) config.closedLoop.p(driveKp);
      return config;
    }

    SparkMaxConfig turnConfig(DriveConstants.ModuleHardware hardware) {
      var config = new SparkMaxConfig();
      config.inverted(hardware.turnInverted()).idleMode(IdleMode.kBrake);
      if (hardware.turnCurrentLimitAmps() != null)
        config.smartCurrentLimit(hardware.turnCurrentLimitAmps());
      if (Double.isFinite(turnPositionRadiansPerMotorRotation))
        config.encoder.positionConversionFactor(turnPositionRadiansPerMotorRotation);
      if (Double.isFinite(turnVelocityRadPerSecPerRpm))
        config.encoder.velocityConversionFactor(turnVelocityRadPerSecPerRpm);
      config
          .closedLoop
          .feedbackSensor(com.revrobotics.spark.FeedbackSensor.kPrimaryEncoder)
          .positionWrappingEnabled(true);
      if (Double.isFinite(turnKp)) config.closedLoop.p(turnKp);
      return config;
    }
  }

  /** Retained for the focused reference-mode tests. */
  static void initializeSteeringReference(
      SteeringReferenceMode mode, DoubleConsumer setTurnEncoderPosition) {
    if (mode == SteeringReferenceMode.MANUAL_ZERO) setTurnEncoderPosition.accept(0.0);
  }
}
