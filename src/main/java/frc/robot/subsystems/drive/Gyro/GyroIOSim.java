package frc.robot.subsystems.drive.Gyro;

import org.wpilib.math.geometry.Rotation2d;

public class GyroIOSim implements GyroIO {
  private Rotation2d yaw = Rotation2d.ZERO;

  public void setYaw(Rotation2d value) {
    yaw = value;
  }

  @Override
  public void updateInputs(GyroIOInputs in) {
    in.yaw = yaw;
    in.connected = true;
  }
}
