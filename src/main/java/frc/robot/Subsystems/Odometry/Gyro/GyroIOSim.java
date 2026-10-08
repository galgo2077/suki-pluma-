package frc.robot.Subsystems.Odometry.Gyro;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOSim implements GyroIO {
  private Rotation2d yaw = new Rotation2d();
  private double yawVelocityRadPerSec;

  public void setYaw(Rotation2d value) {
    yaw = value;
  }

  public void setYawVelocity(double value) {
    yawVelocityRadPerSec = value;
  }

  @Override
  public void updateInputs(GyroIOInputs in) {
    in.yaw = yaw;
    in.yawVelocityRadPerSec = yawVelocityRadPerSec;
    in.connected = true;
  }
}
