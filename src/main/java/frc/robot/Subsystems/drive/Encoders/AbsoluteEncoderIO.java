package frc.robot.Subsystems.drive.Encoders;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Common absolute steering reference; module mapping and offsets are assigned in Phase 2. */
public interface AbsoluteEncoderIO {
  @AutoLog
  class AbsoluteEncoderIOInputs {
    public boolean connected;
    public Rotation2d absolutePosition = new Rotation2d();
  }

  default void updateInputs(AbsoluteEncoderIOInputs inputs) {}
}
