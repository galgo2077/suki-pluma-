package frc.robot.Subsystems.Odometry.Gyro;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;

/** TODO HARDWARE: assign the Pigeon CAN ID before real construction. */
public final class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;

  public GyroIOPigeon2(Pigeon2 pigeon) {
    this.pigeon = pigeon;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    var yaw = pigeon.getYaw();
    inputs.connected = yaw.getStatus().isOK();
    inputs.yaw =
        Rotation2d.fromRadians(
            yaw.getValue().in(Radians)); // TODO CALIBRATE: yaw sign, mounting, and zeroing
    inputs.yawVelocityRadPerSec =
        pigeon
            .getAngularVelocityZWorld()
            .getValue()
            .in(RadiansPerSecond); // TODO CALIBRATE: angular-rate sign
  }
}
