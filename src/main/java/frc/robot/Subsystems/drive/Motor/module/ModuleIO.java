package frc.robot.Subsystems.drive.Motor.module;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  class ModuleIOInputs {
    public double drivePositionMeters;
    public double driveVelocityMetersPerSec;
    public Rotation2d turnPosition = new Rotation2d();
    public double turnVelocityRadPerSec;
    public Rotation2d absoluteTurnPosition = new Rotation2d();
    public double driveCurrentAmps;
    public double turnCurrentAmps;
    public boolean absoluteEncoderConnected;
    public boolean fault;
    public boolean connected;
  }

  default void updateInputs(ModuleIOInputs inputs) {}

  default void setDesired(double metersPerSec, Rotation2d angle) {}

  default void stop() {}
}
