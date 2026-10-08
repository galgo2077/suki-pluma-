package frc.robot.Subsystems.drive.Encoders;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;

/** TODO HARDWARE: assign FL, FR, and BL CANcoder IDs before real construction. */
public final class AbsoluteEncoderIOCANcoder implements AbsoluteEncoderIO {
  private final CANcoder encoder;

  public AbsoluteEncoderIOCANcoder(CANcoder encoder) {
    this.encoder = encoder;
  }

  @Override
  public void updateInputs(AbsoluteEncoderIOInputs inputs) {
    var position = encoder.getAbsolutePosition();
    inputs.connected = position.getStatus().isOK();
    inputs.absolutePosition = Rotation2d.fromRadians(position.getValue().in(Radians));
  }
}
