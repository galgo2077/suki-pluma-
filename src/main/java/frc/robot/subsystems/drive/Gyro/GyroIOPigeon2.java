package frc.robot.subsystems.drive.Gyro;

import static org.wpilib.units.Units.Radians;

import com.ctre.phoenix6.hardware.Pigeon2;
import org.wpilib.math.geometry.Rotation2d;

/** Pigeon2 adapter. Its CAN ID and construction are deferred to Phase 2. */
public final class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;

  public GyroIOPigeon2(Pigeon2 pigeon) {
    this.pigeon = pigeon;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    var yaw = pigeon.getYaw();
    inputs.connected = yaw.getStatus().isOK();
    inputs.yaw = Rotation2d.fromRadians(yaw.getValue().in(Radians));
  }
}
