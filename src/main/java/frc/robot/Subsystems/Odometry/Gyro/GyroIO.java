package frc.robot.Subsystems.Odometry.Gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  class GyroIOInputs {
    public Rotation2d yaw = new Rotation2d();
    public double yawVelocityRadPerSec;
    public boolean connected = true;
  }

  default void updateInputs(GyroIOInputs in) {}
}
