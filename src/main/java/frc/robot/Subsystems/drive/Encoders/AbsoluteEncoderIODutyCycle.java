package frc.robot.Subsystems.drive.Encoders;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

/** Thrifty Through Bore Encoder connected to a roboRIO DIO channel. */
public final class AbsoluteEncoderIODutyCycle implements AbsoluteEncoderIO {
  private final DutyCycleEncoder encoder;

  public AbsoluteEncoderIODutyCycle(int dioChannel) {
    encoder = new DutyCycleEncoder(dioChannel);
  }

  @Override
  public void updateInputs(AbsoluteEncoderIOInputs inputs) {
    inputs.connected = encoder.isConnected();
    inputs.absolutePosition = Rotation2d.fromRotations(encoder.get());
  }
}
