package frc.robot.subsystems.drive.Encoders;

import static org.wpilib.units.Units.Radians;

import com.ctre.phoenix6.hardware.CANcoder;
import org.wpilib.math.geometry.Rotation2d;

/** CTRE CANcoder adapter. The caller constructs CANcoder only after Phase 2 assigns its ID. */
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
