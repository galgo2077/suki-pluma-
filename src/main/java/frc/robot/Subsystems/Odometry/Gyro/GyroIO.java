package frc.robot.Subsystems.Odometry.Gyro;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Rotation2d;

public interface GyroIO {
  @AutoLog
  class GyroIOInputs {
    public Rotation2d yaw = Rotation2d.ZERO;
    public double yawVelocityRadPerSec;
    public boolean connected = true;
  }

  default void updateInputs(GyroIOInputs in) {}
}
