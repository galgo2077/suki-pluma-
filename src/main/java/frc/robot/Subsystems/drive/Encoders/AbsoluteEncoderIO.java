package frc.robot.Subsystems.drive.Encoders;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Rotation2d;

/** Common absolute steering reference; module mapping and offsets are assigned in Phase 2. */
public interface AbsoluteEncoderIO {
  @AutoLog
  class AbsoluteEncoderIOInputs {
    public boolean connected;
    public Rotation2d absolutePosition = Rotation2d.ZERO;
  }

  default void updateInputs(AbsoluteEncoderIOInputs inputs) {}
}
